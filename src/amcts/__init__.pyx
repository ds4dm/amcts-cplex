# distutils: language = c++

import multiprocessing as mp
import threading as tr
from queue import Queue

from .comm import Pipe
from .worker import CplexWorker

import numpy as np

from .comm cimport Pipe, pipe_t
# from libc.stdio cimport printf

cdef extern from "amcts.hpp" namespace "amcts" nogil:
    cdef cppclass Edge:
        int id
        int visits
        int pending_updates
        float score
        float prior
        Node * state
    cdef cppclass Node:
        Edge * actions
        int * action_vars
        int n_actions
    # Cython volatile workaround
    cdef cppclass EdgeVptr:
        Edge * p
        void set(Edge * _p);
    # Cython volatile workaround
    cdef cppclass NodeVptr:
        Node * p
        void set(Node * _p);
    cdef cppclass Tree:
        Edge root
        Worker * workers
        # Edge read_action(int * path, int path_size)  # Cython volatile workaround
        Tree(int n_workers, int max_depth)
    cdef cppclass Worker:
        Worker(Tree * _tree)
        void set_pipes(pipe_t p_in, pipe_t p_out)
        void set_seed(int seed)
        void set_c(float c)
        void run()
        void stop()
        int randint(const int & min, const int & max)

cdef class MCTS:

    cdef int n_jobs
    cdef Tree * tree
    cdef object orders_queue
    cdef object mip_filename
    cdef object rollout_strategy
    cdef int score_limit

    def __init__(self, mip_filename, int n_jobs=1, int max_depth=-1, int score_limit=-1, rollout_strategy='default'):
        self.n_jobs = n_jobs
        self.mip_filename = mip_filename
        self.rollout_strategy = rollout_strategy
        self.score_limit = score_limit

    def __cinit__(self, mip_filename, int n_jobs=1, int max_depth=-1, *args, **kwargs):
        self.tree = new Tree(n_jobs, max_depth)

    def __dealloc__(self):
        del self.tree

    def get_action_info(self, path=[]):

        cdef int i
        cdef EdgeVptr a_p

        a_p.set(&(self.tree.root))
        var = -1

        for i in path:
            if a_p.p.state == NULL:
                raise Exception("Tree path not expanded yet.");
            if i < 0 or i >= a_p.p.state.n_actions:
                raise Exception("Invalid tree path.");
            var = a_p.p.state.action_vars[i]
            a_p.set(&(a_p.p.state.actions[i]))

        return (var, a_p.p.prior, a_p.p.score, a_p.p.visits)

    def get_state_info(self, path=[]):

        cdef int i, n_actions
        cdef NodeVptr s_p

        s_p.set(self.tree.root.state)

        # cdef np.ndarray[int, mode="c", ndim=1] c_path = np.asarray(path, dtype=int)
        #
        # if len(path) == 0:
        #     prev_a = self.tree.read_action(NULL, 0)
        # else:
        #     prev_a = self.tree.read_action(&c_path[0], len(path))

        if s_p.p == NULL:
            raise Exception("Tree is empty.");

        for i in path:
            if i < 0 or i >= s_p.p.n_actions:
                raise Exception("Invalid tree path.");
            s_p.set(s_p.p.actions[i].state)
            if s_p.p == NULL:
                raise Exception("Tree path not expanded yet.");

        vars = np.zeros((s_p.p.n_actions, ), dtype=int)
        priors = np.zeros((s_p.p.n_actions, ), dtype=float)
        scores = np.zeros((s_p.p.n_actions, ), dtype=float)
        visits = np.zeros((s_p.p.n_actions, ), dtype=int)

        for i in range(s_p.p.n_actions):
            vars[i] = s_p.p.action_vars[i]
            priors[i] = s_p.p.actions[i].prior
            scores[i] = s_p.p.actions[i].score
            visits[i] = s_p.p.actions[i].visits

        return (vars, priors, scores, visits)

    def run(self, int n_iters=1000, float c=np.sqrt(2)):

        cdef unsigned int seed

        n_jobs = min(self.n_jobs, n_iters)

        # open master <-> slaves communication channels
        self.orders_queue = Queue(maxsize=n_jobs)

        worker_threads = []

        # start workers
        for i in range(n_jobs):
            seed = <unsigned int> np.random.randint(0, np.iinfo(np.int32).max + 1)
            t = tr.Thread(target=self.run_worker, args=(i, seed, c))
            t.start()
            worker_threads.append(t)

        # print("[m] tree->root.state = {0:x}".format(<unsigned long> self.tree.root.state));
        # print("[m] tree->root.visits = {}".format(self.tree.root.visits));

        # send tasks to workers
        for i in range(n_iters):
            self.orders_queue.put('run')

        # wait for all tasks to be processed
        self.orders_queue.join()

        # tell workers to stop
        for i in range(n_jobs):
            self.orders_queue.put('stop')

        # wait for workers to terminate
        for t in worker_threads:
            t.join()

        # print("All slaves are done.")
        # if self.tree.root.state != NULL:
        #     for i in range(self.tree.root.state.n_actions):
        #         print("dive: {} ({} visits)".format(i, self.tree.root.state.actions[i].visits));
        #
        # print("[m] tree->root.state = {0:x}".format(<unsigned long> self.tree.root.state));
        # print("[m] tree->root.visits = {}".format(self.tree.root.visits));
        # print("[m] tree->root.state.n_actions = {}".format(self.tree.root.state.n_actions));

    def run_worker(self, int i, unsigned int seed, int c):

        cdef Worker * worker = &(self.tree.workers[i])

        # open communication pipes
        p1 = Pipe()
        p2 = Pipe()
        p1.open()
        p2.open()

        # initialize random numbers generator
        worker.set_seed(seed)
        # print("[t] worker seed {}".format(seed))

        # start CPLEX process
        seed = <unsigned int> worker.randint(np.iinfo(np.int32).min, np.iinfo(np.int32).max)
        proc = mp.Process(target=CplexWorker(self.mip_filename, self.rollout_strategy, self.score_limit).run, args=(p1, p2, seed), daemon=True)
        proc.start()

        # run MCTS routine
        p2.init_in()
        p1.init_out()
        worker.set_pipes(p2._p(), p1._p())
        worker.set_c(c)

        while True:
            order = self.orders_queue.get()

            # print("[t] received order '{}'".format(order))

            if order == 'run':
                with nogil:
                    worker.run()
                self.orders_queue.task_done()

            elif order == 'stop':
                worker.stop()
                self.orders_queue.task_done()
                break

            else:
                raise ValueError("Unknown order '{}'".format(order))

            print("[t] {} simulations so far".format(self.tree.root.visits - self.tree.root.pending_updates))

        # wait for CPLEX process to terminate
        proc.join()

        # close communication pipes
        p1.close()
        p2.close()
