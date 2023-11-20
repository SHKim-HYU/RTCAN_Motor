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

// PCI/E-FD
#define DEVICE1 "/dev/rtdm/pcan0"
#define DEVICE2 "/dev/rtdm/pcan1"

#define XDDP_PORT 0	/* [0..CONFIG-XENO_OPT_PIPE_NRDEV - 1] */

RT_TASK ft_task;
RT_TASK motor_task;
RT_TASK xddp_writer;

PCANDevice can1, can2;

Motor_CiA402 motor;

using namespace std;

unsigned int cycle_ns = 1000000; // 1 ms

unsigned char data_field[16];

short raw_data[6] = { 0 };
unsigned short temp;
unsigned DF=50, DT=1000;
float ft_array[6];

void ft_run(void *arg)
{
    RTIME beginCycle, prevCycle;
    unsigned long periodCycle = 0;
    // CAN Setup
    CANDevice::Config_t config;
    config.mode_fd = 0; // 0: CAN2.0 Mode, 1: CAN-FD Mode
    config.bitrate = 1e6; //1mbps
    config.d_bitrate = 2e6; //2mbps
    config.sample_point = .875; //87.5% 
    config.d_sample_point = 0.6; //60%
    config.clock_freq = 80e6; // 80mhz // Read from driver?  
    

    if(!can1.Open(DEVICE1, config, false))
    {
        std::cout << "Unable to open CAN Device" << std::endl;
        // exit(-2);
        return;
    }

    // Setup Filters
    can1.ClearFilters(); // Clear Existing/Reset.  Filters are saved on the device hardware.  Must make sure to clear
    can1.AddFilter(1, 2); // Only Listen to messages on id 0x01, 0x02.  

    int res1, res2;
    CANDevice::CAN_msg_t TxFrame;
    CANDevice::CAN_msg_t RxFrame1;
    CANDevice::CAN_msg_t RxFrame2;

    can1.Status();

    TxFrame.id = 0x64;
    TxFrame.length = 8;
    for(int i=0; i<8; i++) TxFrame.data[i]=0x00;

    // Read Data Output Rate
    TxFrame.data[0] = SET_DATA_RATE;
    TxFrame.data[1] = FT_RATE_1000HZ; // 1000Hz

    can1.Send(TxFrame);

    // Read Once
    TxFrame.data[0] = READ_START;
    
    RxFrame1.length = 8;
    RxFrame2.length = 8;

    can1.Send(TxFrame);
    beginCycle = rt_timer_read();
    rt_task_set_periodic(NULL, TM_NOW, 1*cycle_ns);
    while (1) {
        rt_task_wait_period(NULL); //wait for next cycle
        
        prevCycle = beginCycle;
        beginCycle = rt_timer_read();

        periodCycle = (unsigned long) beginCycle - prevCycle;
        rt_printf("xddp-RT looptime: %lius\n", periodCycle/1000);
        rt_printf("[Force] x: %f, y: %f, z: %f\n",  ft_array[0], ft_array[1], ft_array[2]);
        rt_printf("[Torque] x: %f, y: %f, z: %f\n\n",  ft_array[3], ft_array[4], ft_array[5]);

        res2 = can1.Receive(RxFrame2);
        res1 = can1.Receive(RxFrame1);

        if (res1 == 1 && res2 == 1)
        {
            //CANbus data to Torque data
            for(int i = 0; i<6; i++)
            {
                data_field[i] = RxFrame1.data[i];
                data_field[i+8] = RxFrame2.data[i];
            }
            
            for(int idx = 0; idx<6; idx++)
            {
                temp = data_field[2*idx+1]*256;
                temp += data_field[2*idx+2];

                raw_data[idx] = (signed short) temp;
            }

            // Set Force/Torque Original
            for(int n = 0; n<3; n++)
            {
                ft_array[n] = ((float)raw_data[n]) / (float)DF;
                ft_array[n+3] = ((float)raw_data[n+3]) / (float)DT;
            }
        }
    }
    can1.Close();
}

void motor_run(void *arg)
{
    CANDevice::Config_t config;
    config.mode_fd = 0; // 0: CAN2.0 Mode, 1: CAN-FD Mode
    config.bitrate = 1e6; //1mbps
    config.d_bitrate = 2e6; //2mbps
    config.sample_point = .875; //87.5% 
    config.d_sample_point = 0.6; //60%
    config.clock_freq = 80e6; // 80mhz // Read from driver?  
    
    motor.activate_all(DEVICE2, config);

    float rate_current;

    rate_current = ((float)motor.SDO_RATE_CURRENT(1))/1000.f;

    printf("rate current: %f\n", rate_current);


    rt_task_set_periodic(NULL, TM_NOW, cycle_ns);
    while (1) {
        rt_task_wait_period(NULL); //wait for next cycle
        printf("rate current: %f\n", rate_current);
    }
}

void runQtApplication(int argc, char* argv[]) {
  QApplication a(argc, argv);
  // style our application with custom dark style
  QApplication::setStyle(new DarkStyle);

  // create frameless window (and set windowState or title)
  FramelessWindow framelessWindow;

  // create our mainwindow instance
  MainWindow *mainWindow = new MainWindow;
  // add the mainwindow to our custom frameless window
  framelessWindow.resize(1600,600);
  framelessWindow.setContent(mainWindow);
  framelessWindow.show();
  a.exec();
}

void signal_handler(int signum)
{
    rt_task_delete(&ft_task);
    rt_task_delete(&motor_task);
    rt_task_delete(&xddp_writer);
    printf("Servo drives Stopped!\n");
    exit(1);
}

int main(int argc, char *argv[])
{
    // Perform auto-init of rt_print buffers if the task doesn't do so
    rt_print_init(0, NULL);

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /* Avoids memory swapping for this program */
    mlockall(MCL_CURRENT|MCL_FUTURE);

    cpu_set_t cpuset_qt, cpuset_rt1, cpuset_rt2;
    CPU_ZERO(&cpuset_qt);
    CPU_ZERO(&cpuset_rt1);  
    CPU_ZERO(&cpuset_rt2);  

    CPU_SET(6, &cpuset_qt);  
    CPU_SET(7, &cpuset_rt1);  
    CPU_SET(5, &cpuset_rt2);  
    
    // std::thread qtThread(runQtApplication, argc, argv);
    // pthread_t pthread = qtThread.native_handle();
    // int rc = pthread_setaffinity_np(pthread, sizeof(cpu_set_t), &cpuset_qt);

    rt_task_create(&ft_task, "ft_task", 0, 99, 0);
    rt_task_set_affinity(&ft_task, &cpuset_rt1);
    // rt_task_start(&ft_task, &ft_run, NULL);

    rt_task_create(&motor_task, "motor_task", 0, 99, 0);
    rt_task_set_affinity(&ft_task, &cpuset_rt2);
    rt_task_start(&motor_task, &motor_run, NULL);

    // Must pause here
    pause();
    // qtThread.join();

    // Finalize
    signal_handler(0);

    return 0;
}
#endif
