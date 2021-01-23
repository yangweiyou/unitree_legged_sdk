/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>

#include <rbdl/addons/urdfreader/urdfreader.h>
#include <yaml-cpp/yaml.h>
#include <signal.h>
#include "task/walk_task.hpp"

//#define PRINT_MESSAGE

using namespace UNITREE_LEGGED_SDK;

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics::Addons;
using namespace legged_robot;

const uint joint_num = 12;
const uint foot_num = 4;


shared_ptr<WalkTask> walk;

/**
 * @brief Define the function to be called when ctrl-c (SIGINT) is sent to process
 *
 * @param signum signal Ctrl+C
 * @return void
 */
void signal_callback_handler ( int signum )
{
    walk->Shut();
    exit ( signum ); // Terminate program
}

class Custom
{
public:
    Custom ( uint8_t level ) : safe ( LeggedType::A1 ), udp ( level ) {
        udp.InitCmdData ( cmd );

        /* Get Yaml Paras */
        string legged_robot_path = "/home/unitree/legged_robot_packages/legged_robot"; //getenv ( "LEGGED_ROBOT_PATH" );
        yaml = YAML::LoadFile ( legged_robot_path + "/yaml/param.yaml" );
        yaml_prt = make_shared<YAML::Node> ( yaml );

        /* Get URDF Model */
        string urdf_file = legged_robot_path + "/urdf/a1.urdf";
        URDFReadFromFile ( urdf_file.c_str(), &a1, true, false );
        a1_prt = make_shared<Model> ( a1 );

//         walk = make_shared<WalkTask> ( make_shared<Model>(a1), make_shared<YAML::Node>(yaml) );
        walk = make_shared<WalkTask> ( a1_prt, yaml_prt );
        t = 0.0;
        jmap= {FL_0, FL_1, FL_2, FR_0, FR_1, FR_2, RL_0, RL_1, RL_2, RR_0, RR_1, RR_2};
        q = VectorNd::Zero ( joint_num );
        dq = VectorNd::Zero ( joint_num );

        t1 = 0.0;
        open_loop_control = yaml["open_loop_control"].as<bool>();
    }

    ~Custom() {
    }

    void UDPSend();
    void UDPRecv();
    void RobotControl();

    Safety safe;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    int motiontime = 0;
    float dt = 0.001;     // 0.001~0.01

    Model a1;
    YAML::Node yaml;
    shared_ptr<YAML::Node> yaml_prt;
    shared_ptr<Model> a1_prt;
    legged_robot::JointCmd j;
    double t;
    vector<int> jmap;
    VectorNd q, dq;

    double t1;
    double t_home = 10.0;
    bool init_home = false;
    bool init_walk = false;
    double initPos[joint_num];
    double targetPos[joint_num] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3,
                                   0.0, 0.67, -1.3, -0.0, 0.67, -1.3
                                  };

    bool open_loop_control;
};

void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{
    udp.Send();
}

double jointLinearInterpolation ( double initPos, double targetPos, double rate )
{
    double p;
    rate = std::min ( std::max ( rate, 0.0 ), 1.0 );
    p = initPos* ( 1-rate ) + targetPos*rate;
    return p;
}

void Custom::RobotControl()
{
    motiontime++;
    udp.GetRecv ( state );
    // printf("%d\n", motiontime);

    if ( motiontime >= 10 ) {
        if ( motiontime <= uint ( t_home/dt ) ) {
            t1+=dt;

            if ( !init_home ) {
                for ( int j=0; j<joint_num; j++ ) {
                    initPos[j] = state.motorState[j].q;
                }
                init_home = true;
            }

            double percent = t1/t_home;
            for ( int j=0; j<joint_num; j++ ) {
                cmd.motorCmd[j].q = jointLinearInterpolation ( initPos[j], targetPos[j], percent ); //lastPos[j]* ( 1-percent ) + targetPos[j]*percent;
                cmd.motorCmd[j].dq = 0;
                cmd.motorCmd[j].Kp = 100;
                cmd.motorCmd[j].Kd = 5;
                cmd.motorCmd[j].tau = 0.0f;
            }

            // gravity compensation
            cmd.motorCmd[FR_0].tau = -0.65f;
            cmd.motorCmd[FL_0].tau = +0.65f;
            cmd.motorCmd[RR_0].tau = -0.65f;
            cmd.motorCmd[RL_0].tau = +0.65f;

        } else {
            t += dt;

            for ( uint i=0; i<jmap.size(); i++ ) {
                q[i] = state.motorState[jmap[i]].q;
                dq[i] = state.motorState[jmap[i]].dq;
            }

            VectorNd contact ( foot_num );
            contact << state.footForce[FL_],state.footForce[FR_],state.footForce[RL_],state.footForce[RR_];

            Eigen::Quaterniond imu ( state.imu.quaternion[0], state.imu.quaternion[1], state.imu.quaternion[2], state.imu.quaternion[3] );

            if ( !init_walk ) {
                walk->Init ( q, dq, contact, &imu );
                init_walk = true;
            }

            if ( open_loop_control ) {
                j = walk->Run();
            } else {
                j = walk->Run ( q, dq, contact, &imu );
            }

//             for ( uint i=0; i<jmap.size(); i++ ) {
// //                 cmd.motorCmd[jmap[i]].mode = 0x0A;
// //                 cmd.motorCmd[jmap[i]].q = PosStopF; //j.pos[i];
// //                 cmd.motorCmd[jmap[i]].dq = VelStopF; // j.vel[i];
// //                 cmd.motorCmd[jmap[i]].Kp = 0; //5;
// //                 cmd.motorCmd[jmap[i]].Kd = 0; //1;
// //                 cmd.motorCmd[jmap[i]].tau = j.tau[i];
//                 cmd.motorCmd[jmap[i]].mode = 0x0A;
//                 cmd.motorCmd[jmap[i]].q = j.pos[i];
//                 cmd.motorCmd[jmap[i]].dq = j.vel[i];
//                 cmd.motorCmd[jmap[i]].Kp = 100; //5;
//                 cmd.motorCmd[jmap[i]].Kd = 5; //1;
//                 cmd.motorCmd[jmap[i]].tau = j.tau[i];
//             }
            for ( uint i=0; i<foot_num; i++ ) {
                cmd.motorCmd[jmap[i*3+0]].mode = 0x0A;
                cmd.motorCmd[jmap[i*3+0]].Kp = yaml["joint_control"]["hip_roll"]["kp"].as<double>();
                cmd.motorCmd[jmap[i*3+0]].Kd = yaml["joint_control"]["hip_roll"]["kd"].as<double>();
                cmd.motorCmd[jmap[i*3+0]].q = j.pos[i*3+0];
                cmd.motorCmd[jmap[i*3+0]].dq = j.vel[i*3+0];
                cmd.motorCmd[jmap[i*3+0]].tau = j.tau[i*3+0];
                cmd.motorCmd[jmap[i*3+1]].mode = 0x0A;
                cmd.motorCmd[jmap[i*3+1]].Kp = yaml["joint_control"]["hip_pitch"]["kp"].as<double>();
                cmd.motorCmd[jmap[i*3+1]].Kd = yaml["joint_control"]["hip_pitch"]["kd"].as<double>();
                cmd.motorCmd[jmap[i*3+1]].q = j.pos[i*3+1];
                cmd.motorCmd[jmap[i*3+1]].dq = j.vel[i*3+1];
                cmd.motorCmd[jmap[i*3+1]].tau = j.tau[i*3+1];
                cmd.motorCmd[jmap[i*3+2]].mode = 0x0A;
                cmd.motorCmd[jmap[i*3+2]].Kp = yaml["joint_control"]["knee"]["kp"].as<double>();
                cmd.motorCmd[jmap[i*3+2]].Kd = yaml["joint_control"]["knee"]["kd"].as<double>();
                cmd.motorCmd[jmap[i*3+2]].q = j.pos[i*3+2];
                cmd.motorCmd[jmap[i*3+2]].dq = j.vel[i*3+2];
                cmd.motorCmd[jmap[i*3+2]].tau = j.tau[i*3+2];
            }
        }

        safe.PositionLimit ( cmd );
        safe.PowerProtect ( cmd, state, 8 );
    }

    udp.SetSend ( cmd );
}

int main ( void )
{
    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    /* Register signal and signal handler */
    signal ( SIGINT, signal_callback_handler ); // put behind, otherwise will be overwirtten by others such as ROS

    Custom custom ( LOWLEVEL );
    InitEnvironment();
    LoopFunc loop_control ( "control_loop", custom.dt,    boost::bind ( &Custom::RobotControl, &custom ) );
    LoopFunc loop_udpSend ( "udp_send",     custom.dt, 3, boost::bind ( &Custom::UDPSend,      &custom ) );
    LoopFunc loop_udpRecv ( "udp_recv",     custom.dt, 3, boost::bind ( &Custom::UDPRecv,      &custom ) );

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while ( 1 ) {
        sleep ( 10 );
    };

    return 0;
}
