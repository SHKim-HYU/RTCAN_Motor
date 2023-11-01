#include <iostream>
#include <thread>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

// XDDP 파이프로 데이터를 쓰는 함수
void write_to_pipe(int fd) {
    char msg[] = "Hello RT";
    while (true) {
        if (write(fd, msg, sizeof(msg)) < 0) {
            perror("Failed to write to the XDDP pipe");
            break;
        }
        sleep(1); // 1초 대기
    }
}

// XDDP 파이프에서 데이터를 읽는 함수
void read_from_pipe(int fd) {
    char buf[10];
    while (true) {
        if (read(fd, buf, sizeof(buf)) > 0) {
            std::cout << "Received: " << buf << std::endl;
        }
    }
}

int main() {
    int fd;

    // XDDP 파이프 열기
    fd = open("/proc/xenomai/registry/rtipc/xddp/myXDDPPipe", O_RDWR);
    if (fd < 0) {
        perror("Failed to open XDDP pipe");
        return 1;
    }

    // 스레드 생성 및 실행
    std::thread writer_thread(write_to_pipe, fd);
    std::thread reader_thread(read_from_pipe, fd);

    // 스레드가 끝나길 기다림
    writer_thread.join();
    reader_thread.join();

    close(fd);
    return 0;
}
