"""
area.py

Defines abstract and concrete geometric areas used for collision detection,
navigation goals, and rendering in a 2D environment.

Areas provide a unified interface for:
- Collision checks
- Center queries
- Nearest-point queries
- Rendering

Concrete implementations include rectangular and circular areas.
"""

import pygame
from abc import ABC, abstractmethod
import math

class Area:
    """
    Abstract base class for all navigable or collidable areas.

    Subclasses must implement collision detection, geometry queries,
    updating logic, and rendering behavior.
    """
    pos : tuple
    size : tuple
    color = None

    def __init__(self, color):
        self.color = color

    @abstractmethod
    def check_collision(self, pos : tuple[float, float]) -> bool:
        pass

    @abstractmethod
    def get_center(self) -> tuple[float, float]:
        pass

    @abstractmethod
    def get_nearest(self, pos) -> tuple[float, float]:
        pass

    @abstractmethod
    def update(self) -> None:
        pass

    @abstractmethod
    def render(self, screen : pygame.Surface, offset = (0,0), scale = 1, render_path = False) -> None:
        pass

class RectArea(Area):
    """
    Axis-aligned rectangular area.
    """
    def __init__(self, pos, size, color):
        super().__init__(color)
        self.pos = pos
        self.size = size
        

    def check_collision(self, pos):
        return pos[0] >= self.pos[0] and pos[0] <= self.pos[0] + self.size[0] and pos[1] >= self.pos[1] and pos[1] <= self.pos[1] + self.size[1]

    def get_center(self):
        return (self.pos[0] + self.size[0] / 2, self.pos[1] + self.size[1] / 2)

    def get_nearest(self, pos):
        pass

    def update(self):
        pass

    def render(self, screen, offset = (0,0), scale = 1, render_path = False):
        rect = pygame.Rect(((self.pos[0] - offset[0]) * scale, (self.pos[1] - offset[1]) * scale), (self.size[0] * scale, self.size[1] * scale))
        pygame.draw.rect(screen, self.color, rect)

class CircleArea(Area):
    """
    Circular area defined by a center point and radius.
    """
    def __init__(self, center, radius, color):
        super().__init__(color)
        self.center = center
        self.radius = radius
        

    def check_collision(self, pos):
        dist = math.sqrt((pos[0] - self.center[0]) ** 2 + (pos[1] - self.center[1]) ** 2)
        return dist <= self.radius
        return 

    def get_center(self):
        return self.center

    def get_nearest(self, pos):
        #nearest is just direction from center to pos scaled to radius plus center
        pass

    def update(self):
        pass

    def render(self, screen, offset = (0,0), scale = 1, render_path = False):
        
        pygame.draw.circle(screen, self.color, ((self.center[0] - offset[0]) * scale, (self.center[1] - offset[1]) * scale), self.radius * scale)




