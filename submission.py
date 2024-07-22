from func_timeout import FunctionTimedOut

from Agent import Agent, AgentGreedy
from WarehouseEnv import WarehouseEnv, manhattan_distance
import random
from func_timeout import func_timeout, FunctionTimedOut
import math
import time



# TODO: section a : 3
'''
def smart_heuristic(env: WarehouseEnv, robot_id: int):
    robot = env.get_robot(robot_id)
    if robot.battery <= 3 and robot_id == 1:
        x = 5
    other_robot = env.get_robot((robot_id + 1) % 2)
    pos_packages = [package.position for package in env.packages if package.on_board]
    robot1_to_package = 10
    robot2_to_package = 10
    robot1_to_dest = 10
    robot2_to_dest = 10
    pick_up = 0

    if robot.package is None:
        robot1_to_package = min([manhattan_distance(robot.position, pos) for pos in pos_packages])
    if other_robot.package is None:
        robot2_to_package = min([manhattan_distance(other_robot.position, pos) for pos in pos_packages])

    if robot.package is not None:
        robot1_to_dest = manhattan_distance(robot.position, robot.package.destination)
        pick_up = 10

    if other_robot.package is not None:
        robot2_to_dest = manhattan_distance(robot.position, robot.package.destination)

    print("robot1_to_dest ", robot1_to_dest, "robot1_to_package ", robot1_to_package, "sum ", "pick_up ", pick_up,
          (robot.battery - other_robot.battery) + (robot.credit - other_robot.credit) * 1000 + \
          (10 / (robot1_to_dest + 0.001)) + (10 / (robot1_to_package + 0.001)) + pick_up
          )
    return (robot.battery - other_robot.battery) + (robot.credit - other_robot.credit)*1000 + \
            (10/(robot1_to_dest+0.001)) + (10/(robot1_to_package+0.001)) + pick_up - (robot1_to_dest-robot2_to_dest)
'''

def smart_heuristic_aux(env: WarehouseEnv, robot_id: int):
    robot = env.get_robot(robot_id)
    #if env.done():
    #    return robot.credit * 300
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
        '''
        last_action = None
        try:
            # Pass the function as a callable object to func_timeout
            action = func_timeout(time_limit, lambda: self.iterative_run_step(env, agent_id))
            last_action = action
        except FunctionTimedOut:
            print("Processing took too long and was aborted.")
        return last_action
        '''
        depth = 0
        start_time = time.time()
        action = 'move north'
        while (time.time() - start_time) < time_limit:
            value, action = self.rb_min_max(env, agent_id, agent_id, depth, 'move north')
            depth += 1
        return action
    '''
    def iterative_run_step(self, env: WarehouseEnv, agent_id):
        depth = 0
        action = "move north"
     
        
        try:
            value, action = self.rb_min_max(env, agent_id, agent_id, depth, 'move north')
            depth += 1
            return action
    '''

    def rb_min_max(self, env: WarehouseEnv, my_agent_id, curr_agent_id, depth, act):
        if env.done() or depth == 0:
            return smart_heuristic(env, my_agent_id), act

        operators, children = self.successors(env, curr_agent_id)

        if curr_agent_id == my_agent_id:
            curr_max = -math.inf
            op_max = ""
            for child, op in zip(children, operators):
                v, _ = self.rb_min_max(child, my_agent_id, (curr_agent_id + 1) % 2, depth - 1, op)
                if v > curr_max:
                    curr_max = v
                    op_max = op
            return curr_max, op_max

        else:
            curr_min = math.inf
            op_min = ""
            for child, op in zip(children, operators):
                v, _ = self.rb_min_max(child, my_agent_id, (curr_agent_id + 1) % 2, depth - 1, op)
                if v < curr_min:
                    curr_min = v
                    op_min = op
            return curr_min, op_min


class AgentAlphaBeta(Agent):
    # TODO: section c : 1
    def run_step(self, env: WarehouseEnv, agent_id, time_limit):
        raise NotImplementedError()


class AgentExpectimax(Agent):
    # TODO: section d : 1
    def run_step(self, env: WarehouseEnv, agent_id, time_limit):
        raise NotImplementedError()


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