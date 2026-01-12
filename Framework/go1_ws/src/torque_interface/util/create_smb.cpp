#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <iostream>
#include <cstring>

int main() {
    const char* shm_names[] = {"/joint_state_buffer", "/torque_command_buffer"};
    const size_t shm_size = 4096;  // Adjust this size to your actual buffer structs

    for (auto name : shm_names) {
        int fd = shm_open(name, O_CREAT | O_RDWR, 0666);
        if (fd < 0) {
            std::cerr << "Failed to create shared memory " << name << std::endl;
            return 1;
        }
        if (ftruncate(fd, shm_size) == -1) {
            std::cerr << "Failed to set size for " << name << std::endl;
            return 1;
        }
        void* ptr = mmap(nullptr, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (ptr == MAP_FAILED) {
            std::cerr << "Failed to mmap " << name << std::endl;
            return 1;
        }
        memset(ptr, 0, shm_size);  // Zero initialize the buffer
        munmap(ptr, shm_size);
        close(fd);
        std::cout << "Created and initialized shared memory " << name << std::endl;
    }

    std::cout << "Dummy shared memory created for testing. You can now run your nodes." << std::endl;
    return 0;
}
