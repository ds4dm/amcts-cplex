#include <math.h>
#include <limits>

#include "amcts.hpp"
#include "constants.h"
#include "pipe.h"

namespace amcts {

    Edge::Edge() {
        id = -1;
        state = NULL;
        prev_state = NULL;
        prior = -1;
        visits = 0;
        pending_updates = 0;
        score = 0;}

    Edge::~Edge() {}

    Node::Node(volatile Edge * _prev_action, int _n_actions) {
        prev_action = _prev_action;
        n_actions = _n_actions;

        actions = new Edge[n_actions];
        action_vars = new int[n_actions];

        for (int i=0; i<n_actions; i++) {
            action_vars[i] = -1;
            actions[i].id = i;
            actions[i].prev_state = this;
        }

        worst_score_id = -1;
    }

    Node::~Node() {
        delete[] actions;
        delete[] action_vars;
    }

    // Edge Tree::read_action(int * path, int path_size) {
    //     // Cython volatile workaround
    //     Edge volatile * a = &root;
    //
    //     for (int i=0; i<path_size; i++) {
    //         if (a->state == NULL) {
    //             throw std::string("Path does not exist in the tree.");
    //         }
    //         if (path[i] < 0 || path[i] >= a->state->n_actions) {
    //             throw std::string("Path does not exist in the tree.");
    //         }
    //         a = &(a->state->actions[i]);
    //     }
    //     return (Edge) *a;
    // }

    // Cython volatile workaround
    void EdgeVptr::set(Edge volatile * _p) {
        p = _p;
    }

    // Cython volatile workaround
    void NodeVptr::set(Node volatile * _p) {
        p = _p;
    }

    Tree::Tree(int n_workers, int _max_depth): max_depth(_max_depth) {
        workers = new Worker[n_workers];
        for (int i=0; i<n_workers; i++) {
            workers[i].set_tree(this);
        }
    }

    Tree::~Tree() {
        delete[] workers;
    }

    Worker::Worker() {
        tree = NULL;
        generator = NULL;
        set_seed(0);
    }

    Worker::~Worker() {
        delete generator;
    }

    void Worker::set_tree(Tree * _tree) {
        tree = _tree;
    }

    void Worker::set_pipes(pipe_t _p_in, pipe_t _p_out) {
        p_in = _p_in;
        p_out = _p_out;
    }

    void Worker::set_seed(int seed) {
        delete generator;
        generator = new std::mt19937(seed);
    }

    void Worker::set_c(float _c) {
        c = _c;
    }

    void Worker::run() {
        int msg, n_actions, score, i;
        Edge volatile * a, * a_next;

        a = &(tree->root);

        // rollin phase
        msg = MSG_START;
        pipe_write(&p_out, &msg);
        a_next = pick_next_action_lockfree(a);
        i = 0;
        while (a_next != NULL && (tree->max_depth < 0 || i < tree->max_depth)) {
            a = a_next;
            a_next = pick_next_action_lockfree(a);
            // action id -> variable id
            msg = a->prev_state->action_vars[a->id];
            pipe_write(&p_out, &msg);
            i++;
        }

        // rollout phase (CPLEX process)
        msg = MSG_SWITCH_ROLLOUT;
        pipe_write(&p_out, &msg);
        pipe_read(&p_in, &n_actions);
        if (n_actions > 0) {
            expand_lockfree(a, n_actions);
        };

        msg = MSG_GET_SCORE;
        pipe_write(&p_out, &msg);
        pipe_read(&p_in, &score);

        backprop_lockfree(a, (float) score);
    }

    void Worker::stop() {
        int msg = MSG_STOP;
        pipe_write(&p_out, &msg);
    }

    volatile Edge * Worker::pick_next_action_lockfree(volatile Edge * prev_action) {
        int i, next, visits, parent_visits;
        float score, worst_score, reward, ucb1, best_ucb1;
        volatile Node * state = prev_action->state;

        // increase visits count during rollin (lock-free exploration)
        // lockfree: do not reorder instructions here !!
        prev_action->visits++;
        prev_action->pending_updates++;

        if (state == NULL) {
            // leaf node
            return NULL;
        }

        // random
        // next = randint(0, state->n_actions - 1);

        // UCT
        best_ucb1 = 0;
        next = 0;
        parent_visits = prev_action->visits;
        // printf("[t] parent_visits %d\n", parent_visits);
        for (i=0; i<state->n_actions; i++) {

            // lockfree: do not reorder instructions here !!
            visits = state->actions[i].visits;
            score = state->actions[i].score;
            if (state->worst_score_id == -1) {
                worst_score = score;
            }
            else {
                worst_score = state->actions[state->worst_score_id].score;
            }

            if (visits == 0) {
                // no visit yet -> force exploration (no prior yet)
                next = i;
                break;
            }

            if (worst_score == 0 || worst_score == score) {
                // assume 0/x=0, even for x=0
                reward = 0;
            }
            else {
                reward = (worst_score - score) / worst_score;
                reward = reward < 0 ? 0 : reward;  // ensure positive rewards
            }

            ucb1 = reward + c * sqrt(log(parent_visits) / visits);
            if (ucb1 >= best_ucb1) {
                best_ucb1 = ucb1;
                next = i;
            }

            // printf("[t] var %d: visits %d score %.02f reward %.02f ucb1 %.02f\n", i, visits, score, reward, ucb1);
        }

        return &(state->actions[next]);
    }

    void Worker::expand_lockfree(volatile Edge * action, int n_actions) {
        Node * new_state = node_pool.construct(action, n_actions);

        // obtain variable ids
        for (int i=0; i<n_actions; i++) {
            pipe_read(&p_in, &(new_state->action_vars[i]));
        }

        // permute variables ids (random tie breaks)
        std::shuffle(new_state->action_vars, new_state->action_vars + new_state->n_actions, *generator);

        action->state = new_state;
    }

    void Worker::backprop_lockfree(volatile Edge * action, float score) {
        int tmp_visits;
        volatile Node * parent;

        /*if (action->prev_state != NULL) {
            printf("[t] var %d (%d): prev_score %.02f score %.02f visits %d\n",\
                action->id, action->prev_state->action_vars[action->id],\
                action->score, score, action->visits);
        }*/

        while (true) {

            // lockfree: do not reorder instructions here !!
            tmp_visits = action->visits;
            // visits updated during rollin, tmp_visits >= pending_updates >= 1 guaranteed
            tmp_visits = tmp_visits - action->pending_updates;
            action->score = (action->score * tmp_visits + score) / (tmp_visits + 1);
            action->pending_updates--;

            parent = action->prev_state;
            if (parent == NULL) {
                break;
            }

            // state worst score update (for normalization)
            if (parent->worst_score_id == -1 \
                || action->score > parent->actions[parent->worst_score_id].score) {
                    parent->worst_score_id = action->id;
            }

            action = parent->prev_action;
        }
    }

    // https://stackoverflow.com/questions/21237905/how-do-i-generate-thread-safe-uniform-random-numbers
    /* Thread-safe function that returns a random number between min and max (inclusive).
    This function takes ~142% the time that calling rand() would take. For this extra
    cost you get a better uniform distribution and thread-safety. */
    int Worker::randint(const int & min, const int & max) {
        std::uniform_int_distribution<int> dist(min, max);
        return dist(*generator);
    }
}
