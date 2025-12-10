import pygame
import pygame.gfxdraw
import math
import util, area
import problem, search


class Agent:
    speed = 0.5
    count = 0

    def __init__(self, pos, env, goal, pather, color=(0, 0, 255)):
        self.pos = pos
        self.env = env
        self.goal = goal
        self.pather = pather
        self.color = color
        self.index = Agent.count
        # NEW: Use index as priority (lower index is higher priority)
        self.priority = Agent.count
        Agent.count += 1
        self.path_cache = []
        self.steps = 0

    def get_next_move(self):

        # Check if our path is ready from the pathfinder
        if self.index in self.pather.complete.keys():
            self.path_cache = self.pather.complete[self.index]
            # Remove from complete dict and waiting set
            del self.pather.complete[self.index]
            if self.index in self.pather.waiting:
                self.pather.waiting.remove(self.index)

        # If we have no path and aren't waiting for one, request a new path
        if len(self.path_cache) == 0 and self.index not in self.pather.waiting:
            # Create problem from CURRENT position
            prob = problem.ContinuousNavigation(self.pos, self.env, self.goal, 1000, 1000)

            # Agent's index is its priority. This is the key change for group pathfinding.
            self.pather.queue_path(self.index, prob)
            return (0, 0, 0)  # Wait for path

        if len(self.path_cache) == 0:
            return (0, 0, 0)  # Still waiting

        return self.path_cache.pop(0)

    def update(self):
        if self.goal.check_collision(self.pos):
            return True

        self.steps += 1

        dir = self.get_next_move()
        if len(dir) == 3:
            return False
        

        dir_length = util.length(dir)
        self.pos = (self.pos[0] + (dir[0] / dir_length) * self.speed, self.pos[1] + (dir[1] / dir_length) * self.speed)
        return False

    def render(self, screen, offset, scale, render_path=False):
        pygame.draw.circle(screen, self.color, ((self.pos[0] - offset[0]) * scale, (self.pos[1] - offset[1]) * scale),
                           0.5 * scale)

        if not render_path:
            return
        hold_x = self.pos[0]
        hold_y = self.pos[1]

        for node in self.path_cache:
            hold_x += node[0]
            hold_y += node[1]
            pygame.draw.circle(screen, (200, 200, 255), ((hold_x - offset[0]) * scale, (hold_y - offset[1]) * scale),
                               0.2 * scale)