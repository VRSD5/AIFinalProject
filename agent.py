import pygame
import pygame.gfxdraw
import math
import util, area
import problem, search


class Agent:
    speed = 0.5

    path_cache = []

    def __init__(self, pos, env, goal, method, color = (0,0,255)):
        self.pos = pos
        self.env = env
        self.goal = goal
        self.method = method
        self.color = color
        

    def get_next_move(self):

        
        
        if len(self.path_cache) == 0:
            prob = problem.ContinuousNavigation(self.pos, self.env, self.goal, 1000, 1000)
            self.path_cache = self.method(prob)

        if len(self.path_cache) == 0:
            return (0, 0, 0)

        return self.path_cache.pop(0)

        

    
    def update(self):
        if self.goal.check_collision(self.pos):
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