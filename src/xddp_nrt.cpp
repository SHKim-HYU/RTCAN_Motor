#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <net/if.h>
#include <netinet/in.h>

#define XDDP_PORT 0x1225 // 예제 XDDP 포트

int main(void)
{
    int sock;
    struct sockaddr_ll saddr;
    char buf[10];
    int ifindex;

    // RAW 소켓 생성
    sock = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    if (sock < 0) {
        perror("socket");
        exit(1);
    }

    // 인터페이스 인덱스 가져오기
    ifindex = if_nametoindex("rteth0"); // Xenomai 가상 네트워크 인터페이스

    // 소켓 주소 설정
    memset(&saddr, 0, sizeof(saddr));
    saddr.sll_family = AF_PACKET;
    saddr.sll_ifindex = ifindex;
    saddr.sll_protocol = htons(ETH_P_ALL);

    if (bind(sock, (struct sockaddr *)&saddr, sizeof(saddr)) < 0) {
        perror("bind");
        exit(1);
    }

    // 데이터 수신
    if (recv(sock, buf, sizeof(buf), 0) < 0) {
        perror("recv");
        exit(1);
    }

    printf("Received message: %s\n", buf);

    // 소켓 닫기
    close(sock);
    return 0;
}
