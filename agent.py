"""
agent.py

Defines multiple agent types for 2D navigation and pathfinding experiments.
Agents differ in how they compute movement, including:
- Standard A* search
- Deferred (asynchronous) planning
- Simple greedy motion
- Flow-field navigation
- Hierarchical A*
- Coordinated multi-agent planning with reservations

Dependencies:
- pygame (rendering)
- util (vector math)
- area (collision and goal regions)
- problem (navigation problem definitions)
- search, hastar (pathfinding algorithms)
- flowField (flow-field navigation)
- group (multi-agent coordination utilities)
"""


import pygame
import util, area
import problem, search, flowField, hastar, group


class Agent:
    """
    Base agent class implementing A*-based navigation.

    Agents move continuously toward a goal while avoiding obstacles.
    Subclasses override path planning or movement behavior.
    """
    speed = 0.5
    count = 0

    

    def __init__(self, pos, env, goal, color = (0,0,255)):
        self.pos = pos
        self.env = env
        self.goal = goal
        self.color = color
        self.index = Agent.count
        Agent.count += 1
        self.path_cache = []
        self.steps = 0

    def get_next_move(self) -> tuple:
        if len(self.path_cache) == 0:
            prob = problem.ContinuousNavigation(self.pos, self.env, self.goal)
            self.path_cache = search.astar_search(prob)
            

        if len(self.path_cache) == 0:
            return (0, 0, 0)

        return self.path_cache.pop(0) # type: ignore

        
    def update(self):
        if self.goal.check_collision(self.pos):
            return True
        
        self.steps += 1

        dir = self.get_next_move()
        if len(dir) == 3:
            return False

        #dir = (goal_pos[0] - self.pos[0], goal_pos[1] - self.pos[1])
        dir_length = util.length(dir)
        self.pos = (self.pos[0] + (dir[0] / dir_length) * self.speed, self.pos[1] + (dir[1] / dir_length) * self.speed)
        return False

    def render(self, screen, offset, scale, render_path = False):
        pygame.draw.circle(screen, self.color, ((self.pos[0] - offset[0]) * scale, (self.pos[1] - offset[1]) * scale), max(1, 0.5 * scale))
        #pygame.gfxdraw.pixel(screen, math.floor((self.pos[0] - offset[0]) * scale), math.floor((self.pos[1] - offset[1]) * scale), (0,0,255))

        if not render_path:
            return
        hold_x = self.pos[0]
        hold_y = self.pos[1]

        for node in self.path_cache:
            hold_x += node[0]
            hold_y += node[1]
            pygame.draw.circle(screen, (200,200,255), ((hold_x - offset[0]) * scale, (hold_y - offset[1]) * scale), 0.2 * scale)

class DefferedAgent(Agent):
    """
    Agent that defers path planning to an external path manager.

    Useful for demos to prevent 1 minute of black screen with high A* agent or distance
    """

    speed = 0.5
    count = 0

    

    def __init__(self, pos, env, goal, pather, color = (0,0,255)):
        self.pos = pos
        self.env = env
        self.goal = goal
        self.pather = pather
        self.color = color
        self.index = Agent.count
        Agent.count += 1
        self.path_cache = []
        self.steps = 0

    def get_next_move(self):
        if len(self.path_cache) == 0 and not self.index in self.pather.waiting:
            prob = problem.ContinuousNavigation(self.pos, self.env, self.goal)
            self.pather.queue_path(self.index, prob)
        elif self.index in self.pather.complete.keys():
            self.path_cache = self.pather.complete[self.index]
            

        if len(self.path_cache) == 0:
            return (0, 0, 0)

        return self.path_cache.pop(0)

        

    
    def update(self):
        if self.goal.check_collision(self.pos):
            return True
        
        self.steps += 1

        dir = self.get_next_move()
        if len(dir) == 3:
            return False

        #dir = (goal_pos[0] - self.pos[0], goal_pos[1] - self.pos[1])
        dir_length = util.length(dir)
        self.pos = (self.pos[0] + (dir[0] / dir_length) * self.speed, self.pos[1] + (dir[1] / dir_length) * self.speed)
        return False




class simpleAgent(Agent):
    """
    Very simple greedy agent that moves directly toward the goal center.
    """
    speed = 0.5
    count = 0

    

    def __init__(self, pos, env, goal, color = (0,0,255)):
        self.pos = pos
        self.env = env
        self.goal = goal
        self.color = color
        self.index = Agent.count
        self.steps = 0
        
        Agent.count += 1
        self.path_cache = []
        

    def get_next_move(self):

        if self.pos[1] - self.goal.get_center()[1] > 1:
            return (0, -0.5)
        elif self.pos[1] - self.goal.get_center()[1] < -1:
            return (0, 0.5)
        if self.pos[0] - self.goal.get_center()[0] > 1:
            return (-0.5, 0)
        elif self.pos[0] - self.goal.get_center()[0] < -1:
            return (0.5, 0)
        
        return (0, 0, 0)
        

    
    def update(self):
        if self.goal.check_collision(self.pos):
            return True
        
        self.steps += 1

        for i in self.env:
            if i.check_collision(self.pos):
                return True
        
        dir = self.get_next_move()
        if len(dir) == 3:
            return False

        #dir = (goal_pos[0] - self.pos[0], goal_pos[1] - self.pos[1])
        dir_length = util.length(dir)
        self.pos = (self.pos[0] + (dir[0] / dir_length) * self.speed, self.pos[1] + (dir[1] / dir_length) * self.speed)
        return False



class FlowAgent(Agent):
    """
    Agent that follows a precomputed flow field.
    """
    speed = 0.5
    count = 0

    

    def __init__(self, pos, env, goal, flow, color = (0,0,255)):
        self.pos = pos
        self.env = env
        self.goal = goal
        self.flow = flow
        self.color = color
        self.index = Agent.count
        Agent.count += 1
        self.path_cache = []
        self.steps = 0

    
    def update(self):
        if self.goal.check_collision(self.pos):
            return True
        
        self.steps += 1
        
        dir = self.flow.get_move_at(self.pos)

        
        if len(dir) == 3:
            return False

        #dir = (goal_pos[0] - self.pos[0], goal_pos[1] - self.pos[1])
        dir_length = util.length(dir)
        self.pos = (self.pos[0] + (dir[0] / dir_length) * self.speed, self.pos[1] + (dir[1] / dir_length) * self.speed)

        for object in self.env:
            if object.check_collision(self.pos):
                center = object.get_center()
                if abs(center[0] - self.pos[0]) > abs(center[1] - self.pos[1]):
                    self.pos = (self.pos[0], self.pos[1] + -2 * dir[1])
                else:
                    self.pos = (self.pos[0] + -2 * dir[0], self.pos[1])
                break

        return False


class HAAgent(Agent):
    """
    Agent using hierarchical A* for pathfinding.
    """
    speed = 0.5
    count = 0

    def __init__(self, pos, env, goal, color=(0, 0, 255)):
        self.pos = pos
        self.env = env
        self.goal = goal
        self.color = color
        self.index = HAAgent.count
        HAAgent.count += 1
        self.path_cache = []
        self.steps = 0

    def get_next_move(self):
        if len(self.path_cache) == 0:
            prob = problem.ContinuousNavigation(self.pos, self.env, self.goal)
            self.path_cache = hastar.hierarchical_astar_search(prob)

        if len(self.path_cache) == 0:
            return (0, 0, 0)

        return self.path_cache.pop(0)

    def update(self):
        if self.goal.check_collision(self.pos):
            return True

        self.steps += 1

        dir = self.get_next_move()
        if len(dir) == 3:
            return False

        dir_length = util.length(dir)
        if dir_length > 0:
            self.pos = (self.pos[0] + (dir[0] / dir_length) * self.speed,
                        self.pos[1] + (dir[1] / dir_length) * self.speed)
        return False
    

class CoordinatedAgent(Agent):
    """
    Multi-agent coordinated planner using a reservation system.

    Prevents agents from colliding by reserving future positions in time.
    """
    speed = 0.5
    count = 0

    def __init__(self, pos, env, goal, color=(0, 0, 255)):
        self.pos = pos
        self.env = env
        self.goal = goal
        self.color = color
        self.index = CoordinatedAgent.count
        CoordinatedAgent.count += 1
        self.path_cache = []
        self.steps = 0
        self.time_step = 0  # Tracks time for reservation system

    def get_next_move(self):
        if len(self.path_cache) == 0:
            prob = problem.ContinuousNavigation(self.pos, self.env, self.goal)

            # Run standard A*
            calculated_path = search.astar_search(prob)

            if calculated_path:
                # Reserve the entire path in the global system
                group.ReservationSystem.reserve_path(self.index, calculated_path, self.pos)
                self.path_cache = calculated_path

            if len(self.path_cache) == 0:
                return (0, 0, 0)

        # Look at the next move in the cache
        next_move = self.path_cache[0]

        # Calculate the potential next position
        dir_length = util.length(next_move)
        if dir_length == 0:
            next_pos = self.pos
        else:
            next_pos = (self.pos[0] + (next_move[0] / dir_length) * self.speed,
                        self.pos[1] + (next_move[1] / dir_length) * self.speed)

        # Check for immediate collision with other agents' future reservations
        # Check time_step + 1 because the move happens between self.time_step and self.time_step + 1
        if group.ReservationSystem.is_reserved(next_pos, self.time_step + 1, self.index):
            # Collision detected: stall for one time step
            return (0, 0, 0)  # Use the "no move" sentinel

        # Move is clear, pop it and return
        return self.path_cache.pop(0)

    def update(self):
        if self.goal.check_collision(self.pos):
            return True

        self.steps += 1
        self.time_step += 1  # Advance time

        dir = self.get_next_move()
        if len(dir) == 3:
            # If (0, 0, 0) is returned, the agent is waiting/stalling.
            return False

        dir_length = util.length(dir)
        if dir_length > 0:
            self.pos = (self.pos[0] + (dir[0] / dir_length) * self.speed,
                        self.pos[1] + (dir[1] / dir_length) * self.speed)
        return False