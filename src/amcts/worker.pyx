# distutils: language = c++

import cplex
import numpy as np

import cplex._internal._constants as cst
from cplex.callbacks import BranchCallback

from math import floor

cdef extern from "constants.h":
    cdef int MSG_START
    cdef int MSG_SWITCH_ROLLOUT
    cdef int MSG_GET_SCORE
    cdef int MSG_STOP

class MCTSCallback(BranchCallback):

    def __init__(self, env):
        super(MCTSCallback, self).__init__(env)
        self.env = env
        self.num_nodes = 1
        self.set_state('rollin')
        self.set_strategy('default')

    def set_pipes(self, p_in, p_out):
        self.p_in = p_in
        self.p_out = p_out

    def set_variables(self, variables):
        self.variables = variables
        self.var_types = self.variables.get_types()

    def set_strategy(self, strategy):
        assert strategy in ('default', 'random'),\
            "Invalid strategy: {}".format(strategy)
        self.strategy = strategy

        if self.strategy == 'random':
            # avoid CPLEX computing branching scores (strong branching etc)
            # https://www.ibm.com/support/knowledgecenter/SSSA5P_12.8.0/ilog.odms.cplex.help/refpythoncplex/html/cplex.callbacks.BranchCallback-class.html
            self.env.parameters.mip.strategy.variableselect.set(cst.CPX_VARSEL_MININFEAS)

        elif self.strategy == 'default':
            self.env.parameters.mip.strategy.variableselect.set(cst.CPX_VARSEL_DEFAULT)

    def set_state(self, state):
        assert state in ('rollin', 'rollout'),\
            "Invalid state: {}".format(state)
        self.state = state

    def set_score_limit(self, score_limit):
        self.score_limit = score_limit

    def backprop(self):

        order = self.p_in.read()

        if order == MSG_SWITCH_ROLLOUT:
            # special case: rolled in the whole tree depth, 0 actions to choose from
            self.p_out.write(0)
            order = self.p_in.read()

        if order == MSG_GET_SCORE:
            print("[p] rollout score: {}".format(self.num_nodes))
            self.p_out.write(self.num_nodes)

        else:
            raise ValueError("Inconsistent order '{}'".format(order))

    def __call__(self):

        # ignore callbacks on non-branching decisions
        if self.get_branch_type() != self.branch_type.variable:
            return

        # ignore CPLEX termitation callback
        if self.get_num_branches() == 0:
            return

        # early stopping criterion
        if self.score_limit >= 0 and self.num_nodes >= self.score_limit:
            self.abort()
            return

        self.num_nodes += 2

        x = self.get_values()
        z = self.get_objective_value()
        feas = self.get_feasibilities()
        ub = self.variables.get_upper_bounds()
        lb = self.variables.get_lower_bounds()

        # branching candidates: infeasible variables only ?
        br_cands = [i for i in range(len(x))
                if feas[i] == self.feasibility_status.infeasible]

        # # branching candidates: all non-continuous, non-implied-feasible and non-fixed variables ?
        # # https://www.ibm.com/developerworks/community/forums/html/topic?id=9b2f163e-5cef-49fe-949d-98c2fa49a455
        # br_cands = [i for i in range(len(x))
        #         if feas[i] != self.feasibility_status.implied_feasible
        #         and self.var_types[i] != self.variables.type.continuous
        #         and lb[i] < ub[i]]

        n_br_cands = len(br_cands)

        if self.state == 'rollin':

            order = self.p_in.read()
            # print("[p] received {}".format(order))

            if order == MSG_SWITCH_ROLLOUT:
                self.p_out.write(n_br_cands)
                for i in range(n_br_cands):
                    self.p_out.write(br_cands[i])
                self.state = 'rollout'
                self.env.parameters.randomseed.set(np.random.randint(
                    self.env.parameters.randomseed.min(),
                    self.env.parameters.randomseed.max() + 1))
                # print("[p] switched to rollout")

            elif order >= 0:
                br_var = order

            else:
                raise ValueError("Inconsistent order '{}'".format(order))

        if self.state == 'rollout':

            if self.strategy == 'default':
                # let CPLEX handle branching
                return

            if self.strategy == 'random':
                # random branching
                br_var = int(br_cands[np.random.randint(n_br_cands)])
                # print("[p] {} chosen".format(br_var))

        # do the branching
        br_lo = floor(x[br_var])

        # integer-feasible variable: branch above ? branch below ?
        if feas[br_var] == self.feasibility_status.feasible:
            # branch above, unless upper bound reached. Arbitrary !
            # branch in direction of objective ?
            if br_lo == ub[br_var]:
                br_lo = br_lo - 1

        self.make_branch(z, variables=[(br_var, "L", br_lo + 1)])
        self.make_branch(z, variables=[(br_var, "U", br_lo)])


cdef class CplexWorker:

    cdef object mip_filename
    cdef object strategy
    cdef int score_limit

    def __init__(self, mip_filename, strategy, int score_limit):
        self.mip_filename = mip_filename
        self.strategy = strategy
        self.score_limit = score_limit

    def run(self, p_in, p_out, unsigned int seed):

        p_in.init_in()
        p_out.init_out()

        np.random.seed(seed)
        # print("[p] worker seed {}".format(seed))

        while(True):
            # print("[p] waiting...")
            order = p_in.read()

            if order == MSG_STOP:
                break

            elif order == MSG_START:

                # print("[p] running cplex...")
                c = cplex.Cplex()

                c.set_log_stream(None)
                c.set_results_stream(None)
                # c.set_error_stream(None)
                # c.set_warning_stream(None)

                # search till optimality
                c.parameters.mip.tolerances.mipgap.set(0.)

                # disable dynamic search
                # https://www.ibm.com/support/knowledgecenter/SSSA5P_12.8.0/ilog.odms.cplex.help/CPLEX/UsrMan/topics/progr_adv/callbacks_basic/16_control_cb_dyn_srch.html
                c.parameters.mip.strategy.search.set(cst.CPX_MIPSEARCH_TRADITIONAL)

                c.read(self.mip_filename)

                # callbacks and parallelism
                # https://www.ibm.com/support/knowledgecenter/SSSA5P_12.8.0/ilog.odms.cplex.help/CPLEX/UsrMan/topics/progr_adv/callbacks_basic/17_deter_parallel_srch.html
                mcts_cb = c.register_callback(MCTSCallback)
                mcts_cb.set_pipes(p_in, p_out)
                mcts_cb.set_variables(c.variables)
                mcts_cb.set_strategy(self.strategy)
                mcts_cb.set_score_limit(self.score_limit);

                c.solve()
                mcts_cb.backprop()

                c.end()

            else:
                raise ValueError("Inconsistent order '{}'".format(order))

        print("[p] stopping.")
        p_in.close()
        p_out.close()
