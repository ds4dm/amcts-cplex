#include <unistd.h>
#include <stdio.h>

#include "pipe.h"

int pipe_open(pipe_t * p) {
    int ret = pipe(p->handle);
    // printf("opened pipe %d %d\n", p->handle[0], p->handle[1]);
    return ret;
}

int pipe_init_in(pipe_t * p) {
    int ret = close(p->handle[1]);
    p->dir = 0;
    // printf("use pipe %d as input\n", p->handle[p->dir]);
    return ret;
}

int pipe_init_out(pipe_t * p) {
    int ret = close(p->handle[0]);
    p->dir = 1;
    // printf("use pipe %d as output\n", p->handle[p->dir]);
    return ret;
}

int pipe_read(const pipe_t * p, int * data) {
    // printf("reading from pipe %d\n", p->handle[p->dir]);
    int ret = read(p->handle[p->dir], data, sizeof(int));
    // printf("received %d from pipe %d\n", *data, p->handle[p->dir]);
    return ret;
}

int pipe_write(const pipe_t * p, int * data) {
    int ret = write(p->handle[p->dir], data, sizeof(int));
    // printf("sent %d to pipe %d\n", *data, p->handle[p->dir]);
    return ret;
}

int pipe_close(const pipe_t * p) {
    int ret = close(p->handle[p->dir]);
    // printf("closed pipe %d\n", p->handle[p->dir]);
    return ret;
}
