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

// PCI/E-FD
#define DEVICE1 "/dev/rtdm/pcan0"
#define DEVICE2 "/dev/rtdm/pcan1"

#define XDDP_PORT 0	/* [0..CONFIG-XENO_OPT_PIPE_NRDEV - 1] */

RT_TASK ft_task;
RT_TASK motor_task;
RT_TASK xddp_writer;

PCANDevice can1, can2;

using namespace std;

unsigned int cycle_ns = 1000000; // 1 ms

unsigned char data_field[16];

short raw_data[6] = { 0 };
unsigned short temp;
unsigned DF=50, DT=2000;
double ft_array[6];

void ft_run(void *arg)
{
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

    TxFrame.id = 0x64;
    TxFrame.length = 8;
    TxFrame.data[0] = 0x0A;
    TxFrame.data[1] = 0x00;
    TxFrame.data[2] = 0x00;
    TxFrame.data[3] = 0x00;
    TxFrame.data[4] = 0x00;
    TxFrame.data[5] = 0x00;
    TxFrame.data[6] = 0x00;
    TxFrame.data[7] = 0x00;

    RxFrame1.length = 8;
    RxFrame2.length = 8;

    can1.Status();

    can1.Send(TxFrame);

    rt_task_set_periodic(NULL, TM_NOW, cycle_ns);
    while (1) {
        rt_task_wait_period(NULL); //wait for next cycle
        
        res2 = can1.Receive(RxFrame2);
        res1 = can1.Receive(RxFrame1);

        can1.Send(TxFrame);

        if (res1 == 1 && res2 == 1)
        {
            //CANbus data to Torque data
            for(int i = 0; i<6; i++)
            {
                data_field[i] = (unsigned char) RxFrame1.data[i];
                data_field[i+8] = (unsigned char) RxFrame2.data[i];
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
                ft_array[n] = ((float)raw_data[n]) / DF;
                ft_array[n+3] = ((float)raw_data[n+3]) / DT;
            }

            // rt_printf("\nX direction Force: %f   \n",   ft_array[0]);
            // rt_printf("Y direction Force: %f    \n",    ft_array[1]);
            // rt_printf("Z direction Force: %f    \n\n",  ft_array[2]);
            // rt_printf("X direction Torque: %f   \n",    ft_array[3]);
            // rt_printf("Y direction Torque: %f   \n",    ft_array[4]);
            // rt_printf("Z direction Torque: %f   \n\n",  ft_array[5]);
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
    

    if(!can2.Open(DEVICE2, config, false))
    {
        std::cout << "Unable to open CAN Device2" << std::endl;
        // exit(-2);
        return;
    }

    // Setup Filters
    can2.ClearFilters(); // Clear Existing/Reset.  Filters are saved on the device hardware.  Must make sure to clear
    can2.AddFilter(1, 2); // Only Listen to messages on id 0x01, 0x02.  

    CANDevice::CAN_msg_t txmsg;
    CANDevice::CAN_msg_t rxmsg;
    
    txmsg.id = 0x64;
    txmsg.length = 8;
    txmsg.data[0] = 0x0A;
    txmsg.data[1] = 0x00;
    txmsg.data[2] = 0x00;
    txmsg.data[3] = 0x00;
    txmsg.data[4] = 0x00;
    txmsg.data[5] = 0x00;
    txmsg.data[6] = 0x00;
    txmsg.data[7] = 0x00;

    // rxmsg.id = 0x01;
    rxmsg.length = 8;

    can2.Status();

    // can.Send(txmsg);

    rt_task_set_periodic(NULL, TM_NOW, cycle_ns);
    while (1) {
        rt_task_wait_period(NULL); //wait for next cycle
        // can2.Send(txmsg);
        
        // can2.Receive(rxmsg);
        // rt_printf("Device2 \n");
        // rt_printf("id: %d, ",rxmsg.id);
        // rt_printf("data: %X %X %X %X %X %X %X %X\n",rxmsg.data[0],rxmsg.data[1],rxmsg.data[2],rxmsg.data[3],rxmsg.data[4],rxmsg.data[5],rxmsg.data[6],rxmsg.data[7]);

        // can2.Receive(rxmsg);
        // rt_printf("id: %d, ",rxmsg.id);
        // rt_printf("data: %X %X %X %X %X %X %X %X\n",rxmsg.data[0],rxmsg.data[1],rxmsg.data[2],rxmsg.data[3],rxmsg.data[4],rxmsg.data[5],rxmsg.data[6],rxmsg.data[7]);
        // rt_printf("\n\n");
    }
    can2.Close();
}

static void fail(const char *reason)
{
	perror(reason);
	exit(EXIT_FAILURE);
}

void xddp_writer_run(void *arg)
{
    RTIME beginCycle, prevCycle;
    unsigned long periodCycle = 0;

    struct sockaddr_ipc saddr;
	int ret, s, n = 0, len;
	struct timespec ts;
	size_t poolsz;

    rt_task_set_periodic(NULL, TM_NOW, 1*cycle_ns); // 1ms

	s = __cobalt_socket(AF_RTIPC, SOCK_DGRAM, IPCPROTO_XDDP);
	if (s < 0) {
		perror("socket");
		exit(EXIT_FAILURE);
	}

	poolsz = 16384; /* bytes */
	ret = __cobalt_setsockopt(s, SOL_XDDP, XDDP_POOLSZ,
			 &poolsz, sizeof(poolsz));
	if (ret)
		fail("setsockopt");

	memset(&saddr, 0, sizeof(saddr));
	saddr.sipc_family = AF_RTIPC;
	saddr.sipc_port = XDDP_PORT;
	ret = __cobalt_bind(s, (struct sockaddr *)&saddr, sizeof(saddr));
	if (ret)
		fail("bind");
    beginCycle = rt_timer_read();
    while(1) 
    {
        rt_task_wait_period(NULL); //wait for next cycle
        prevCycle = beginCycle;
        beginCycle = rt_timer_read();

        periodCycle = (unsigned long) beginCycle - prevCycle;
        rt_printf("xddp-RT looptime: %lius\n", periodCycle/1000);
        rt_printf("[Force] x: %f, y: %f, z: %f\n",  ft_array[0], ft_array[1], ft_array[2]);
        rt_printf("[Torque] x: %f, y: %f, z: %f\n\n",  ft_array[3], ft_array[4], ft_array[5]);

		ret = __cobalt_sendto(s, ft_array, sizeof(ft_array), 0, NULL, 0);
		// if (ret != sizeof(ft_array))
		// 	fail("sendto");

		// /* Read back packets echoed by the regular thread */
		// ret = __cobalt_recvfrom(s, buf, sizeof(buf), 0, NULL, 0);
		// if (ret <= 0)
		// 	fail("recvfrom");

		// printf("   => \"%.*s\" echoed by peer\n", ret, buf);
	}

	return;
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

    cpu_set_t cpuset_qt, cpuset_rt;
    CPU_ZERO(&cpuset_qt);
    CPU_ZERO(&cpuset_rt);  

    CPU_SET(6, &cpuset_qt);  
    CPU_SET(7, &cpuset_rt);  
    
    std::thread qtThread(runQtApplication, argc, argv);
    pthread_t pthread = qtThread.native_handle();
    int rc = pthread_setaffinity_np(pthread, sizeof(cpu_set_t), &cpuset_qt);

    rt_task_create(&ft_task, "ft_task", 0, 99, 0);
    rt_task_set_affinity(&ft_task, &cpuset_rt);
    rt_task_start(&ft_task, &ft_run, NULL);

    rt_task_create(&motor_task, "motor_task", 0, 98, 0);
    rt_task_start(&motor_task, &motor_run, NULL);

    rt_task_create(&xddp_writer, "xddp_writer", 0, 80, 0);
    rt_task_start(&xddp_writer, &xddp_writer_run, NULL);

    // Must pause here
    pause();
    qtThread.join();

    // Finalize
    signal_handler(0);

    return 0;
}
#endif
