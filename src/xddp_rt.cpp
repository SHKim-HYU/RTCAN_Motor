#include <unistd.h>
#include <stdio.h>
#include <signal.h>
#include <sys/mman.h>
#include <native/task.h>
#include <native/timer.h>
#include <native/pipe.h>

RT_PIPE mypipe;
RT_TASK mytask_writer;
RT_TASK mytask_reader;

void rt_task_writer(void *arg) {
    int num = 0;
    ssize_t size;
    char msg[10];

    while (1) {
        sprintf(msg, "Msg %d", num++);
        size = rt_pipe_write(&mypipe, msg, sizeof(msg), P_NORMAL);
        if (size < 0) {
            printf("Failed to write to RT pipe\n");
            break;
        }
        rt_task_sleep(1000000); // Sleep for 1s
    }
}

void rt_task_reader(void *arg) {
    ssize_t size;
    char msg[10];

    while (1) {
        size = rt_pipe_read(&mypipe, msg, sizeof(msg), TM_INFINITE);
        if (size < 0) {
            printf("Failed to read from RT pipe\n");
            break;
        }
        printf("Received: %s\n", msg);
    }
}

int main(int argc, char* argv[]) {
    mlockall(MCL_CURRENT|MCL_FUTURE);

    if (rt_pipe_create(&mypipe, "myXDDPPipe", P_MINOR_AUTO, 0)) {
        printf("Failed to create RT pipe\n");
        return 1;
    }

    rt_task_create(&mytask_writer, "mytask_writer", 0, 50, 0);
    rt_task_create(&mytask_reader, "mytask_reader", 0, 50, 0);

    rt_task_start(&mytask_writer, &rt_task_writer, NULL);
    rt_task_start(&mytask_reader, &rt_task_reader, NULL);

    pause();  // Wait for Ctrl+C

    rt_task_delete(&mytask_writer);
    rt_task_delete(&mytask_reader);
    rt_pipe_delete(&mypipe);

    return 0;
}
