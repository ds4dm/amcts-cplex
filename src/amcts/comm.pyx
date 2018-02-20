# distutils: language = c++

cdef class Pipe:

    cdef pipe_t _p(self):
        return self.p

    def open(self):
        if pipe_open(&(self.p)):
            raise IOError("Could not obtain pipe handles.")

    def close(self):
        if pipe_close(&(self.p)):
            raise IOError("Could not close pipe handle. Already closed pipe?")

    def init_in(self):
        if pipe_init_in(&(self.p)):
            raise IOError("Could not close output handle.")

    def init_out(self):
        if pipe_init_out(&(self.p)):
            raise IOError("Could not close input handle.")

    def read(self):
        cdef int data
        if pipe_read(&(self.p), &data) != sizeof(int):
            raise IOError("Could not read from pipe.")
        return data

    def write(self, int data):
        if pipe_write(&(self.p), &data) != sizeof(int):
            raise IOError("Could not write to pipe.")
