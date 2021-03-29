/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>

// #include <yaml-cpp/yaml.h>
#include <signal.h>
#include "legged_control.hpp"

//#define PRINT_MESSAGE

using namespace UNITREE_LEGGED_SDK;
using namespace std;

const uint joint_num = 12;
const uint foot_num = 4;

shared_ptr<legged_robot::LeggedControl> control;

void signal_callback_handler ( int signum )
{
    control->Shut();
    exit ( signum ); // Terminate program
}

class Custom
{
public:
    Custom ( uint8_t level ) : safe ( LeggedType::A1 ), udp ( level ) {
        udp.InitCmdData ( cmd );

        /* Get Yaml Paras */
//        string legged_robot_path = getenv ( "LEGGED_ROBOT_PATH" );
        control = make_shared<legged_robot::LeggedControl> ( "/home/unitree/legged_robot_packages/legged_robot" );
        t = 0.0;
        jmap= {FL_0, FL_1, FL_2, FR_0, FR_1, FR_2, RL_0, RL_1, RL_2, RR_0, RR_1, RR_2};
        q = Eigen::VectorXd::Zero ( joint_num );
        dq = Eigen::VectorXd::Zero ( joint_num );
	contact = Eigen::VectorXd::Zero(foot_num);

        t1 = 0.0;
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
    float dt = 0.001; // 0.001, 0.001~0.01

    legged_robot::JointCmd j;
    double t;
    vector<int> jmap;
    Eigen::VectorXd q, dq;
    Eigen::VectorXd contact;
    legged_robot::IMU imu;

    double t1;
    double t_home = 10.0;
    bool init_home = false;
    bool init_control = false;
    double initPos[joint_num];
    double targetPos[joint_num] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3,
                                   0.0, 0.67, -1.3, -0.0, 0.67, -1.3
                                  };
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
        if ( motiontime <= uint ( t_home/dt ) ) { // Homing
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

        } else { // Control
            t += dt;

            for ( uint i=0; i<jmap.size(); i++ ) {
                q[i] = state.motorState[jmap[i]].q;
                dq[i] = state.motorState[jmap[i]].dq;
            }
            contact << state.footForce[FL_],state.footForce[FR_],state.footForce[RL_],state.footForce[RR_];

            imu.quat = Eigen::Quaterniond ( state.imu.quaternion[0], state.imu.quaternion[1], state.imu.quaternion[2], state.imu.quaternion[3] );
            imu.omega << state.imu.gyroscope[0], state.imu.gyroscope[1], state.imu.gyroscope[2];
            imu.accel << state.imu.accelerometer[0], state.imu.accelerometer[1], state.imu.accelerometer[2];

            if ( !init_control ) {
                control->Init ( q, dq, contact, imu );
                init_control = true;
            }

            j = control->Run ( q, dq, contact, imu );

            for ( uint i=0; i<foot_num; i++ ) {
                cmd.motorCmd[jmap[i*3+0]].mode = 0x0A;
                cmd.motorCmd[jmap[i*3+0]].Kp = 100;
                cmd.motorCmd[jmap[i*3+0]].Kd = 5;
                cmd.motorCmd[jmap[i*3+0]].q = j.pos[i*3+0];
                cmd.motorCmd[jmap[i*3+0]].dq = j.vel[i*3+0];
                cmd.motorCmd[jmap[i*3+0]].tau = j.tau[i*3+0];
                cmd.motorCmd[jmap[i*3+1]].mode = 0x0A;
                cmd.motorCmd[jmap[i*3+1]].Kp = 100;
                cmd.motorCmd[jmap[i*3+1]].Kd = 5;
                cmd.motorCmd[jmap[i*3+1]].q = j.pos[i*3+1];
                cmd.motorCmd[jmap[i*3+1]].dq = j.vel[i*3+1];
                cmd.motorCmd[jmap[i*3+1]].tau = j.tau[i*3+1];
                cmd.motorCmd[jmap[i*3+2]].mode = 0x0A;
                cmd.motorCmd[jmap[i*3+2]].Kp = 100;
                cmd.motorCmd[jmap[i*3+2]].Kd = 5;
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
