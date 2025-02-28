from func_timeout import FunctionTimedOut

from Agent import Agent, AgentGreedy
from WarehouseEnv import WarehouseEnv, manhattan_distance
import random
from func_timeout import func_timeout, FunctionTimedOut
import math
import time



# TODO: section a : 3


def smart_heuristic_aux(env: WarehouseEnv, robot_id: int):
    robot = env.get_robot(robot_id)
    if env.done():
        return robot.credit * 300
    other_robot = env.get_robot((robot_id + 1) % 2)
    heuristic_value = 50 * robot.credit
    if robot.package is None:
       dist_to_package = [manhattan_distance(robot.position, package.position) for package in env.packages if package.on_board]
       if robot.battery < min(dist_to_package):
           dist_to_charger = [manhattan_distance(robot.position, charger.position) for charger in env.charge_stations]
           heuristic_value = 100 * (heuristic_value + 25 - min(dist_to_charger))
       else:
           heuristic_value = heuristic_value + 25 - min(dist_to_package)
    else:
        heuristic_value = heuristic_value + 50 - manhattan_distance(robot.position, robot.package.destination)
    return heuristic_value

def smart_heuristic(env: WarehouseEnv, robot_id: int):
    return smart_heuristic_aux(env, robot_id) - smart_heuristic_aux(env, (robot_id + 1) % 2)


class AgentGreedyImproved(AgentGreedy):
    def heuristic(self, env: WarehouseEnv, robot_id: int):
        return smart_heuristic(env, robot_id)


class AgentMinimax(Agent):
    def run_step(self, env: WarehouseEnv, agent_id, time_limit):
        finish_time = time.time() + time_limit - 0.1
        depth = 0
        action = None
        try:
            while True:
                if time.time() >= finish_time:
                    raise Exception("Agent used too much time!")
                _, action = self.rb_min_max(env, agent_id, agent_id, depth, action, finish_time)
                depth += 1
        except:
            return action


    def rb_min_max(self, env: WarehouseEnv, my_agent_id, curr_agent_id, depth, act, finish_time):
        if time.time() >= finish_time:
            raise Exception("Agent used too much time!")
        if env.done() or depth == 0:
            return smart_heuristic(env, my_agent_id), act

        operators, children = self.successors(env, curr_agent_id)

        if curr_agent_id == my_agent_id:
            curr_max = -math.inf
            op_max = ""
            for child, op in zip(children, operators):
                v, _ = self.rb_min_max(child, my_agent_id, (curr_agent_id + 1) % 2, depth - 1, op, finish_time)
                if v > curr_max:
                    curr_max = v
                    op_max = op
            return curr_max, op_max

        else:
            curr_min = math.inf
            op_min = ""
            for child, op in zip(children, operators):
                v, _ = self.rb_min_max(child, my_agent_id, (curr_agent_id + 1) % 2, depth - 1, op, finish_time)
                if v < curr_min:
                    curr_min = v
                    op_min = op
            return curr_min, op_min


class AgentAlphaBeta(Agent):
    def run_step(self, env: WarehouseEnv, agent_id, time_limit):
        finish_time = time.time() + time_limit - 0.1
        depth = 0
        action = None
        try:
            while True:
                if time.time() >= finish_time:
                    raise Exception("Agent used too much time!")
                _, action = self.rb_alpha_beta(env, agent_id, agent_id, depth, action, finish_time, -math.inf, math.inf)
                depth += 1
        except:
            return action


    def rb_alpha_beta(self, env: WarehouseEnv, my_agent_id, curr_agent_id, depth, act, finish_time, alpha, beta):
        if time.time() >= finish_time:
            raise Exception("Agent used too much time!")
        if env.done() or depth == 0:
            return smart_heuristic(env, my_agent_id), act

        operators, children = self.successors(env, curr_agent_id)

        if curr_agent_id == my_agent_id:
            curr_max = -math.inf
            op_max = ""
            for child, op in zip(children, operators):
                v, _ = self.rb_alpha_beta(child, my_agent_id, (curr_agent_id + 1) % 2, depth - 1, op, finish_time, alpha, beta)
                if v > curr_max:
                    curr_max = v
                    op_max = op
                alpha = max(curr_max, alpha)
                if curr_max >= beta:
                    return math.inf
            return curr_max, op_max

        else:
            curr_min = math.inf
            op_min = ""
            for child, op in zip(children, operators):
                v, _ = self.rb_alpha_beta(child, my_agent_id, (curr_agent_id + 1) % 2, depth - 1, op, finish_time, alpha, beta)
                if v < curr_min:
                    curr_min = v
                    op_min = op
                beta = min(curr_min, beta)
                if curr_min <= alpha:
                    return -math.inf
            return curr_min, op_min



class AgentExpectimax(Agent):
    def run_step(self, env: WarehouseEnv, agent_id, time_limit):
        finish_time = time.time() + time_limit - 0.1
        depth = 0
        action = None
        try:
            while True:
                if time.time() >= finish_time:
                    raise Exception("Agent used too much time!")
                _, action = self.rb_expectivemax(env, agent_id, agent_id, depth, action, finish_time)
                depth += 1
        except:
            return action

    def calc_prob(self, legal_operators):
        operators_number = len(legal_operators)
        dict_prob = {}
        move_east_flag = False
        pick_up_flag = False

        if "move east" in legal_operators:
            move_east_flag = True
        if "pick up" in legal_operators:
            pick_up_flag = True

        if move_east_flag and pick_up_flag:
            prob_uniform = 1 / (operators_number + 2)
            for op in legal_operators:
                if op == "move east" or op == "pick up":
                    dict_prob[op] = 2 * prob_uniform
                else:
                    dict_prob[op] = prob_uniform

        elif move_east_flag or pick_up_flag:
            prob_uniform = 1 / (operators_number + 1)
            for op in legal_operators:
                if op == "move east" or op == "pick up":
                    dict_prob[op] = 2 * prob_uniform
                else:
                    dict_prob[op] = prob_uniform
        else:
            prob_uniform = 1 / operators_number
            for op in legal_operators:
                dict_prob[op] = prob_uniform
        return dict_prob

    def rb_expectivemax(self, env: WarehouseEnv, my_agent_id, curr_agent_id, depth, act, finish_time):
        if time.time() >= finish_time:
            raise Exception("Agent used too much time!")
        if env.done() or depth == 0:
            return smart_heuristic(env, my_agent_id), act

        operators, children = self.successors(env, curr_agent_id)

        if curr_agent_id == my_agent_id:
            curr_max = -math.inf
            op_max = ""
            for child, op in zip(children, operators):
                v, _ = self.rb_expectivemax(child, my_agent_id, (curr_agent_id + 1) % 2, depth - 1, op, finish_time)
                if v > curr_max:
                    curr_max = v
                    op_max = op
            return curr_max, op_max

        else:
            operators, children = self.successors(env, curr_agent_id)
            dict_prob = self.calc_prob(operators)
            for child, op in zip(children, operators):
                expective = sum(dict_prob[op] * (self.rb_expectivemax(child, my_agent_id, (curr_agent_id + 1) % 2,
                                                                     depth - 1, op, finish_time)[0]))
            return expective, act




# here you can check specific paths to get to know the environment
class AgentHardCoded(Agent):
    def __init__(self):
        self.step = 0
        # specifiy the path you want to check - if a move is illegal - the agent will choose a random move
        self.trajectory = ["move east","move east","move east","move east","move east","move east","move east"
                           ,"move east","move east","move east","move east","move east","move east","move east"
                           ,"move east","move east","move east","move east","move east","move east","move east"]


    def run_step(self, env: WarehouseEnv, robot_id, time_limit):
        if self.step == len(self.trajectory):
            return self.run_random_step(env, robot_id, time_limit)
        else:
            op = self.trajectory[self.step]
            if op not in env.get_legal_operators(robot_id):
                op = self.run_random_step(env, robot_id, time_limit)
            self.step += 1
            return op

    def run_random_step(self, env: WarehouseEnv, robot_id, time_limit):
        operators, _ = self.successors(env, robot_id)

        return random.choice(operators)