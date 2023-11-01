#include <stdio.h>
#include <unistd.h>
#include <sys/mman.h>
#include <rtdm/rtipc.h> // RTDM (Real-Time Driver Model) for IPC

#define XDDP_PORT 0x1225 // 예제 XDDP 포트

int main(void)
{
    int sock;
    struct sockaddr_ipc saddr;
    struct timespec ts;
    char msg[10];

    // Mlockall을 사용하여 메모리 페이지를 고정
    mlockall(MCL_CURRENT | MCL_FUTURE);

    // XDDP 소켓 생성
    sock = rt_dev_socket(AF_RTIPC, SOCK_DGRAM, IPCPROTO_XDDP);
    if (sock < 0) {
        perror("rt_dev_socket");
        return 1;
    }

    // 소켓 주소 설정
    saddr.sipc_family = AF_RTIPC;
    saddr.sipc_port = XDDP_PORT;
    if (rt_dev_bind(sock, (struct sockaddr *)&saddr, sizeof(saddr)) < 0) {
        perror("rt_dev_bind");
        return 1;
    }

    while(1)
    {
    // 메시지 전송
    snprintf(msg, sizeof(msg), "Hello");
    if (rt_dev_sendto(sock, msg, sizeof(msg), 0, NULL, 0) < 0) {
        perror("rt_dev_sendto");
        return 1;
    }
    }

    // 소켓 닫기
    rt_dev_close(sock);
    return 0;
}