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
#include "ctrl/whole_body_control.hpp"
#include "odom/odometer.hpp"
#include "utils/saveEigen.h"

//#define PRINT_MESSAGE

using namespace UNITREE_LEGGED_SDK;

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics::Addons;
using namespace legged_robot;

const uint joint_num = 12;
const uint foot_num = 4;

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

        odom_prt = make_shared<Odometer> ( a1_prt, yaml_prt );
        control_prt = make_shared<WholeBodyControl> ( a1_prt, yaml_prt );
        first_loop = true;
        t = 0.0;
        jmap= {FL_0, FL_1, FL_2, FR_0, FR_1, FR_2, RL_0, RL_1, RL_2, RR_0, RR_1, RR_2};
        q = VectorNd::Zero ( joint_num );
        dq = VectorNd::Zero ( joint_num );
        Q = VectorNd::Zero ( joint_num+6 );
        DQ = VectorNd::Zero ( joint_num+6 );

        t1 = 0.0;
        logger.resize ( 12, 400 );
    }

    ~Custom() {
        cout << "Save file!" << endl;
        save ( &logger, "/home/unitree/a1_log_data.csv" );
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
    shared_ptr<Odometer> odom_prt;
    shared_ptr<WholeBodyControl> control_prt;
    legged_robot::HighCmd h;
    legged_robot::JointCmd j;
    bool first_loop;
    Eigen::Vector3d com_position0;
    double t;
    vector<int> jmap;
    VectorNd q, dq, Q, DQ;

    double t1;
    double t_home = 10.0;
    bool init = false;
    double initPos[joint_num];
    double targetPos[joint_num] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3,
                                   0.0, 0.67, -1.3, -0.0, 0.67, -1.3
                                  };
    Eigen::MatrixXd logger;
    uint logger_counter = 0;
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

            if ( !init ) {
                for ( int j=0; j<joint_num; j++ ) {
                    initPos[j] = state.motorState[j].q;
                }
                init = true;
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

            Eigen::Quaterniond imu ( state.imu.quaternion[0], state.imu.quaternion[1], state.imu.quaternion[2], state.imu.quaternion[3] );

            VectorNd contact ( foot_num );
            contact << state.footForce[FL_],state.footForce[FR_],state.footForce[RL_],state.footForce[RR_];

            Point base;
            base = odom_prt->Run ( q, dq, contact );

            VectorNd Q ( joint_num+6 );
            VectorNd DQ ( joint_num+6 );
            Q << base.pos, base.euler_pos, q;
            DQ << base.vel, base.euler_vel, dq;
            control_prt->Update ( Q, DQ );

            if ( first_loop ) {
                for ( uint i=0; i<foot_num; i++ ) {
                    Point x;
                    x.pos << control_prt->GetFootPosition ( i );
                    h.traj.push_back ( x );
                    h.contact.push_back ( true );
                }
                Point x;
                x.pos << control_prt->GetCoMPosition();
                h.traj.push_back ( x );
                h.contact.push_back ( false );

                com_position0 = control_prt->GetCoMPosition();
                first_loop = false;
            }

            Point x;
//             x.pos << com_position0 + ( cos ( t*2*M_PI/10.0 )-1 ) *Vector3d ( 0, 0, 0.05 );
//             x.vel << -sin ( t*2*M_PI/10.0 ) *Vector3d ( 0, 0, 0.05*2*M_PI/10.0 );
//             x.acc << -cos ( t*2*M_PI/10.0 ) *Vector3d ( 0, 0, 0.05*2*M_PI/10.0*2*M_PI/10.0 );
	    double T = 10.0;
            x.pos << com_position0 + ( cos ( t/T*2*M_PI )-1 ) *Vector3d ( 0, 0, 0.05 );
            x.vel << -2*M_PI/T*sin ( t/T*2*M_PI )*Vector3d ( 0, 0, 0.05 );
            x.acc << -pow(2*M_PI/T, 2)*cos ( t/T*2*M_PI )*Vector3d ( 0, 0, 0.05 );
            h.traj[foot_num] = x;

            j = control_prt->InverseDynamics ( h );
            static uint print_counter = 0;
            print_counter++;
            if ( print_counter==100 ) {
                print_counter = 0;
#ifdef PRINT_MESSAGE
                cout << "---------------------------------------------" << endl;
                cout << "measured com positon: " << control_prt->GetCoMPosition() << endl;
                cout << "measured base orient: " << base.euler_pos.transpose() << endl;
                cout << "desired com positon: " << x.pos.transpose() << endl;
                cout << "desired base orient: " << x.euler_pos.transpose() << endl;
#endif
                logger.col ( logger_counter ) << control_prt->GetCoMPosition(), base.euler_pos, x.pos, x.euler_pos;
                logger_counter++;
//cout << "logger counter: " << logger_counter << "\t logger cols " << logger.rows() << endl;
                if ( logger_counter>=logger.cols() ) {
                    cout << "Save file!" << endl;
                    save ( &logger, "/home/unitree/a1_log_data.csv" );
                    logger_counter = 0;
                }
            }
//            cout << "position cmd: " << j.pos.transpose() << endl;
//            cout << "torque cmd: " << j.tau.transpose() << endl;

            for ( uint i=0; i<jmap.size(); i++ ) {
                cmd.motorCmd[jmap[i]].mode = 0x0A;
                cmd.motorCmd[jmap[i]].q = j.pos[i];
                cmd.motorCmd[jmap[i]].dq = j.vel[i];
                cmd.motorCmd[jmap[i]].Kp = 0; //5;
                cmd.motorCmd[jmap[i]].Kd = 0; //1;
                cmd.motorCmd[jmap[i]].tau = j.tau[i];
            }
        }

        safe.PositionLimit ( cmd );
        safe.PowerProtect ( cmd, state, 5 );
    }

//     safe.PowerProtect ( cmd, state, 6 );

    udp.SetSend ( cmd );
}

int main ( void )
{
    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

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
