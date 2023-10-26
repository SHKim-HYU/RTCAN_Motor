#ifndef RTCAN_FT_CLIENT_H_
#define RTCAN_FT_CLIENT_H_

#include <stdio.h>
#include "iostream"
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include <rtdk.h>
#include <native/task.h>
#include <native/timer.h>

#include <PCANDevice.h>

// PCI/E-FD
#define DEVICE "/dev/rtdm/pcan1"

RT_TASK can_task;

PCANDevice can;

unsigned int cycle_ns = 1000000; // 1 ms

void can_comm_task(void *arg)
{
    CANDevice::Config_t config;
    config.mode_fd = 0; // 0: CAN2.0 Mode, 1: CAN-FD Mode
    config.bitrate = 1e6; //1mbps
    config.d_bitrate = 2e6; //2mbps
    config.sample_point = .875; //87.5% 
    config.d_sample_point = 0.6; //60%
    config.clock_freq = 80e6; // 80mhz // Read from driver?  
    

    if(!can.Open(DEVICE, config, false))
    {
        std::cout << "Unable to open CAN Device" << std::endl;
        // exit(-2);
        return;
    }

    // Setup Filters
    can.ClearFilters(); // Clear Existing/Reset.  Filters are saved on the device hardware.  Must make sure to clear
    can.AddFilter(1, 2); // Only Listen to messages on id 0x01, 0x02.  

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

    can.Status();

    // can.Send(txmsg);

    rt_task_set_periodic(NULL, TM_NOW, cycle_ns);
    while (1) {
        rt_task_wait_period(NULL); //wait for next cycle
        can.Send(txmsg);
        
        can.Receive(rxmsg);
        rt_printf("id: %d, ",rxmsg.id);
        rt_printf("data: %X %X %X %X %X %X %X %X\n",rxmsg.data[0],rxmsg.data[1],rxmsg.data[2],rxmsg.data[3],rxmsg.data[4],rxmsg.data[5],rxmsg.data[6],rxmsg.data[7]);

        can.Receive(rxmsg);
        rt_printf("id: %d, ",rxmsg.id);
        rt_printf("data: %X %X %X %X %X %X %X %X\n",rxmsg.data[0],rxmsg.data[1],rxmsg.data[2],rxmsg.data[3],rxmsg.data[4],rxmsg.data[5],rxmsg.data[6],rxmsg.data[7]);
        rt_printf("\n\n");
    }
    can.Close();
}


void signal_handler(int signum)
{
    rt_task_delete(&can_task);
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

    rt_task_create(&can_task, "canTask", 0, 99, 0);
    rt_task_start(&can_task, &can_comm_task, NULL);

    // Must pause here
    pause();

    // Finalize
    signal_handler(0);

    return 0;
}
#endif
