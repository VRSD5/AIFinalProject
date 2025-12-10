import pygame
import pygame.gfxdraw
import math
import util, area
import problem, search, flowField


class Agent:
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

    def get_next_move(self):
        if len(self.path_cache) == 0:
            prob = problem.ContinuousNavigation(self.pos, self.env, self.goal, 1000, 1000)
            self.path_cache = search.astar_search(prob)
            

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

    def render(self, screen, offset, scale, render_path = False):
        pygame.draw.circle(screen, self.color, ((self.pos[0] - offset[0]) * scale, (self.pos[1] - offset[1]) * scale), 0.5 * scale)
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
            prob = problem.ContinuousNavigation(self.pos, self.env, self.goal, 1000, 1000)
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

    def render(self, screen, offset, scale, render_path = False):
        pygame.draw.circle(screen, self.color, ((self.pos[0] - offset[0]) * scale, (self.pos[1] - offset[1]) * scale), 0.5 * scale)
        #pygame.gfxdraw.pixel(screen, math.floor((self.pos[0] - offset[0]) * scale), math.floor((self.pos[1] - offset[1]) * scale), (0,0,255))

        if not render_path:
            return
        hold_x = self.pos[0]
        hold_y = self.pos[1]

        for node in self.path_cache:
            hold_x += node[0]
            hold_y += node[1]
            pygame.draw.circle(screen, (200,200,255), ((hold_x - offset[0]) * scale, (hold_y - offset[1]) * scale), 0.2 * scale)


class simpleAgent(Agent):
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

    def render(self, screen, offset, scale, render_path = False):
        

        pygame.draw.circle(screen, self.color, ((self.pos[0] - offset[0]) * scale, (self.pos[1] - offset[1]) * scale), 0.5 * scale)
        #pygame.gfxdraw.pixel(screen, math.floor((self.pos[0] - offset[0]) * scale), math.floor((self.pos[1] - offset[1]) * scale), (0,0,255))

        if not render_path:
            return
        hold_x = self.pos[0]
        hold_y = self.pos[1]

        for node in self.path_cache:
            hold_x += node[0]
            hold_y += node[1]
            pygame.draw.circle(screen, (200,200,255), ((hold_x - offset[0]) * scale, (hold_y - offset[1]) * scale), 0.2 * scale)


class FlowAgent(Agent):
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

    def render(self, screen, offset, scale, render_path = False):
        pygame.draw.circle(screen, self.color, ((self.pos[0] - offset[0]) * scale, (self.pos[1] - offset[1]) * scale), 0.5 * scale)
        #pygame.gfxdraw.pixel(screen, math.floor((self.pos[0] - offset[0]) * scale), math.floor((self.pos[1] - offset[1]) * scale), (0,0,255))

        if not render_path:
            return
        hold_x = self.pos[0]
        hold_y = self.pos[1]

        for node in self.path_cache:
            hold_x += node[0]
            hold_y += node[1]
            pygame.draw.circle(screen, (200,200,255), ((hold_x - offset[0]) * scale, (hold_y - offset[1]) * scale), 0.2 * scale)
