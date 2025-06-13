#include <errno.h>
#include <stddef.h>

void* _sbrk(ptrdiff_t incr) {
    errno = ENOMEM;
    return (void*)-1;
}

int _close(int fd) {
    return -1;
}

int _read(int fd, char* buf, int count) {
    return 0;
}

int _fstat(int fd, void* statbuf) {
    return 0;
}

int _isatty(int fd) {
    return 1;
}

int _lseek(int fd, int offset, int whence) {
    return 0;
}

void _exit(int status) {
    while (1)
        ;
}

int _kill(int pid, int sig) {
    return -1;
}

int _getpid(void) {
    return 1;
}
