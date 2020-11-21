/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>

using namespace std;
using namespace UNITREE_LEGGED_SDK;

uint joint_num = 12;

class Custom
{
public:
    Custom ( uint8_t level ) : safe ( LeggedType::Aliengo ), udp ( level ) {
        udp.InitCmdData ( cmd );
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();

    Safety safe;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    float qInit[3]= {0};
    float qDes[3]= {0};
    float sin_mid_q[3] = {0.0, 1.2, -2.0};
    float Kp[3] = {0};
    float Kd[3] = {0};
    double time_consume = 0;
    int rate_count = 0;
    int sin_count = 0;
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
};

void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{
    udp.Send();
}

void Custom::RobotControl()
{
    motiontime++;
    udp.GetRecv ( state );
    // printf("%d  %f\n", motiontime, state.motorState[FR_2].q);

    // if( motiontime >= 100){
    if ( motiontime >= 10 ) {
        for ( int j=0; j<joint_num; j++ ) {
            cmd.motorCmd[j].q = 0;
            cmd.motorCmd[j].dq = 0;
            cmd.motorCmd[j].Kp = 0;
            cmd.motorCmd[j].Kd = 0;
            cmd.motorCmd[j].tau = 0.0f;
        }

        safe.PositionLimit ( cmd );
        safe.PowerProtect ( cmd, state, 5 );
        // You can uncomment it for position protection
        // safe.PositionProtect(cmd, state, 0.087);
    }

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
