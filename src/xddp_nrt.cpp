#include <iostream>
#include <thread>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

void write_to_pipe(int fd) {
    char msg[] = "Hello RT";
    while (true) {
        if (write(fd, msg, sizeof(msg)) < 0) {
            perror("Failed to write to the XDDP pipe");
            break;
        }
        sleep(1);
    }
}

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

    // XDDP Path
    fd = open("/proc/xenomai/registry/rtipc/xddp/myXDDPPipe", O_RDWR);
    if (fd < 0) {
        perror("Failed to open XDDP pipe");
        return 1;
    }

    std::thread writer_thread(write_to_pipe, fd);
    std::thread reader_thread(read_from_pipe, fd);

    writer_thread.join();
    reader_thread.join();

    close(fd);
    return 0;
}
