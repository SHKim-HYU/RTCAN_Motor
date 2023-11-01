#ifndef RTCAN_MOTOR_CLIENT_H_
#define RTCAN_MOTOR_CLIENT_H_

#include <stdio.h>
#include "iostream"
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include <rtdk.h>
#include <native/task.h>
#include <native/timer.h>

// Peak CAN
#include <PCANDevice.h>

// PCI/E-FD
#define DEVICE1 "/dev/rtdm/pcan0"
#define DEVICE2 "/dev/rtdm/pcan1"

RT_TASK ft_task;
RT_TASK motor_task;

PCANDevice can1, can2;

using namespace std;

unsigned int cycle_ns = 1000000; // 1 ms

void ft_run(void *arg)
{
    CANDevice::Config_t config;
    config.mode_fd = 0; // 0: CAN2.0 Mode, 1: CAN-FD Mode
    config.bitrate = 1e6; //1mbps
    config.d_bitrate = 2e6; //2mbps
    config.sample_point = .875; //87.5% 
    config.d_sample_point = 0.6; //60%
    config.clock_freq = 80e6; // 80mhz // Read from driver?  
    

    if(!can1.Open(DEVICE1, config, false))
    {
        std::cout << "Unable to open CAN Device1" << std::endl;
        // exit(-2);
        return;
    }

    // Setup Filters
    can1.ClearFilters(); // Clear Existing/Reset.  Filters are saved on the device hardware.  Must make sure to clear
    can1.AddFilter(1, 2); // Only Listen to messages on id 0x01, 0x02.  

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

    can1.Status();

    // can.Send(txmsg);

    rt_task_set_periodic(NULL, TM_NOW, cycle_ns);
    while (1) {
        rt_task_wait_period(NULL); //wait for next cycle
        can1.Send(txmsg);
        rt_printf("Device1 \n");
        can1.Receive(rxmsg);
        rt_printf("id: %d, ",rxmsg.id);
        rt_printf("data: %X %X %X %X %X %X %X %X\n",rxmsg.data[0],rxmsg.data[1],rxmsg.data[2],rxmsg.data[3],rxmsg.data[4],rxmsg.data[5],rxmsg.data[6],rxmsg.data[7]);

        can1.Receive(rxmsg);
        rt_printf("id: %d, ",rxmsg.id);
        rt_printf("data: %X %X %X %X %X %X %X %X\n",rxmsg.data[0],rxmsg.data[1],rxmsg.data[2],rxmsg.data[3],rxmsg.data[4],rxmsg.data[5],rxmsg.data[6],rxmsg.data[7]);
        rt_printf("\n\n");
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
        can2.Send(txmsg);
        
        can2.Receive(rxmsg);
        rt_printf("Device2 \n");
        rt_printf("id: %d, ",rxmsg.id);
        rt_printf("data: %X %X %X %X %X %X %X %X\n",rxmsg.data[0],rxmsg.data[1],rxmsg.data[2],rxmsg.data[3],rxmsg.data[4],rxmsg.data[5],rxmsg.data[6],rxmsg.data[7]);

        can2.Receive(rxmsg);
        rt_printf("id: %d, ",rxmsg.id);
        rt_printf("data: %X %X %X %X %X %X %X %X\n",rxmsg.data[0],rxmsg.data[1],rxmsg.data[2],rxmsg.data[3],rxmsg.data[4],rxmsg.data[5],rxmsg.data[6],rxmsg.data[7]);
        rt_printf("\n\n");
    }
    can2.Close();
}

void signal_handler(int signum)
{
    rt_task_delete(&ft_task);
    rt_task_delete(&motor_task);
    printf("Servo drives Stopped!\n");
    exit(1);
}

int main(int argc, char *argv[])
{
    // Perform auto-init of rt_print buffers if the task doesn't do so
    rt_print_auto_init(1);

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /* Avoids memory swapping for this program */
    mlockall(MCL_CURRENT|MCL_FUTURE);

    rt_task_create(&ft_task, "ft_task", 0, 99, 0);
    rt_task_start(&ft_task, &ft_run, NULL);

    rt_task_create(&motor_task, "motor_task", 0, 90, 0);
    rt_task_start(&motor_task, &motor_run, NULL);

    // Must pause here
    pause();

    // Finalize
    signal_handler(0);

    return 0;
}
#endif
