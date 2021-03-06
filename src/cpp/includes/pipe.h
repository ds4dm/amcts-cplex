#pragma once

typedef struct pipe_t {
    int handle[2];
    int dir;
} pipe_t;

int pipe_open(pipe_t * p);
int pipe_init_in(pipe_t * p);
int pipe_init_out(pipe_t * p);
int pipe_close(const pipe_t * p);
int pipe_read(const pipe_t * p, int * data);
int pipe_write(const pipe_t * p, int * data);
