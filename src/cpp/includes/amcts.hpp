#pragma once

#include <random>
#include <boost/pool/object_pool.hpp>
// #include <boost/scoped_array.hpp>

#include "pipe.h"

namespace amcts {

    class Node;
    class Edge {
    public:
        Node volatile * prev_state;
        Node volatile * state;
        int id;
        float prior;
        float score;
        int visits;
        int pending_updates;
        Edge();
        ~Edge();
    };

    class Node {
    public:
        Edge volatile * prev_action;
        Edge volatile * actions;
        int * action_vars;
        int worst_score_id;
        int n_actions;
        Node(Edge volatile * _prev_action, int _n_actions);
        ~Node();
    };

    class Worker;
    class Tree {
    public:
        Edge volatile root;
        int const max_depth;
        Worker * workers;
        // Edge read_action(int * path, int path_size);  // Cython volatile workaround
        Tree(const int n_workers, int const max_depth);
        ~Tree();
    };

    // Cython volatile workaround
    class EdgeVptr {
    public:
        Edge volatile * p;
        void set(Edge volatile * _p);
    };
    class NodeVptr {
    public:
        Node volatile * p;
        void set(Node volatile * _p);
    };

    class Worker {
    public:
        void set_seed(int _seed);
        void set_tree(Tree * _tree);
        void set_pipes(pipe_t _p_in, pipe_t _p_out);
        void set_c(float _c);
        void run();
        void stop();
        int randint(const int & min, const int & max);
        Worker();
        ~Worker();
    private:
        Tree volatile * tree;
        float c;
        pipe_t p_in, p_out;
        std::mt19937 * generator;
        boost::object_pool<Node> node_pool;
        void expand_lockfree(volatile Edge * action, int n_actions);
        void backprop_lockfree(volatile Edge * action, float score);
        volatile Edge * pick_next_action_lockfree(volatile Edge * prev_action);
    };
}
