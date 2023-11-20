#ifndef RTCAN_MOTOR_CLIENT_H_
#define RTCAN_MOTOR_CLIENT_H_

#include <stdio.h>
#include "iostream"
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>

#include <sched.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <rtdm/ipc.h> 

#undef debug
//QT
// #include <thread>
// #include <pthread.h>
#include <QApplication>
#include <QSplitter>
#include <QTreeView>
#include <QListView>
#include <QTableView>
#include <QStandardItemModel>
#include <QScreen>

#include "DarkStyle.h"
#include "framelesswindow.h"
#include "mainwindow.h"
#include <iostream>

#include <QApplication>
#include <QResource>
#include <QTextCodec>

// Peak CAN
#include <PCANDevice.h>
#include <Robotous_FT.h>
#include <MotorCiA402.h>
#include <PropertyDefinition.h>

// PCI/E-FD
#define DEVICE1 "/dev/rtdm/pcan0"
#define DEVICE2 "/dev/rtdm/pcan1"

#define XDDP_PORT 0	/* [0..CONFIG-XENO_OPT_PIPE_NRDEV - 1] */

typedef struct STATE{
	JVec q;
	JVec q_dot;
	JVec q_ddot;
	JVec tau;
	JVec tau_ext;
	JVec e;
	JVec eint;
	JVec G;

	Vector6d x;                           //Task space
	Vector6d x_dot;
	Vector6d x_ddot;
	Vector6d F;
	Vector6d F_CB;
    Vector6d F_ext;
    
    double s_time;
}state;

typedef struct MOTOR_INFO{
    double torque_const[JOINTNUM];
    double gear_ratio[JOINTNUM];
    double rate_current[JOINTNUM];
}Motor_Info;

typedef struct ROBOT_INFO{
	int Position;
	int q_inc[JOINTNUM];
    int dq_inc[JOINTNUM];
	int tau_per[JOINTNUM];
	int statusword[JOINTNUM];
    int modeofop[JOINTNUM];

	JVec q_target;
	JVec qdot_target;
	JVec qddot_target;
	JVec traj_time;
	unsigned int idx;

	STATE act;
	STATE des;
	STATE nom;

    Motor_Info motor;

}ROBOT_INFO;

#endif  // /* RTCAN_MOTOR_CLIENT_H */