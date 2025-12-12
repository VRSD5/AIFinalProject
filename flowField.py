import pygame
import area, util


class FlowField:
    class FlowNode:
        def __init__(self, pos: tuple, target_pos: tuple, target_dir: tuple = None):
            # Calculate direction to target
            self.pos = pos

            # if target_dir == (0,0):
            if target_dir == None:
                self.dir = (target_pos[0] - self.pos[0], target_pos[1] - self.pos[1])
            else:
                self.dir = target_dir
            # else:
            #    self.dir = util.dir_average((target_pos[0] - self.pos[0], target_pos[1] - self.pos[1]), target_dir)

        def get_dir(self):
            return self.dir

        def get_pos(self):
            return self.pos

        def render(self, surface, offset=(0, 0), scale=1):
            pygame.draw.line(surface, (0, 255, 0),
                             ((self.pos[0] - offset[0]) * scale, (self.pos[1] - offset[1]) * scale),
                             ((self.pos[0] + self.dir[0] * 2 - offset[0]) * scale,
                              (self.pos[1] + self.dir[1] * 2 - offset[1]) * scale))
            pygame.draw.circle(surface, (0, 200, 0),
                               ((self.pos[0] - offset[0]) * scale, (self.pos[1] - offset[1]) * scale), 0.25 * scale)

    def __init__(self, goal: area.Area, env, density=5, region=(0, 0, 200, 125)):
        self.goal: area.Area = goal
        self.env = env
        self.density = density
        self.region = region
        self.fit()

    def fit(self):
        # Perform backpropogation
        start = (int(self.goal.get_center()[0]) // self.density * self.density,
                 int(self.goal.get_center()[1]) // self.density * self.density)

        root_node = FlowField.FlowNode(start, self.goal.get_center())

        self.nodes = {start: root_node}
        node_queue = [root_node]

        offsets = ((-self.density, 0), (self.density, 0), (0, self.density), (0, -self.density))

        while len(node_queue) > 0:
            current = node_queue.pop(0)
            # print(current.pos)
            for i in offsets:
                new_pos = (current.pos[0] + i[0], current.pos[1] + i[1])
                if new_pos in self.nodes.keys():
                    continue
                if not (self.region[0] <= new_pos[0] <= self.region[2] and self.region[1] <= new_pos[1] <= self.region[
                    3]):
                    continue
                test = False
                for i in self.env:
                    if i.check_collision(new_pos):
                        test = True
                        self.nodes[new_pos] = FlowField.FlowNode(new_pos, None, (new_pos[0] - i.get_center()[0],
                                                                                 new_pos[1] - i.get_center()[1]))
                        break

                if test:
                    continue
                self.nodes[new_pos] = FlowField.FlowNode(new_pos, current.pos)
                node_queue.append(self.nodes[new_pos])

        # print(self.nodes)

    def get_move_at(self, pos):
        # Find closest nodes and interpolate
        start = (int(pos[0]) // self.density * self.density, int(pos[1]) // self.density * self.density)

        offsets = ((0, 0), (0, self.density), (self.density, 0), (self.density, self.density))

        # dir = (0,0)

        # for i in offsets:
        #     loc = (start[0] + i[0], start[1] + i[1])
        #     if loc in self.nodes:
        #         dir = util.dir_average(dir, self.nodes[loc].dir)

        # if len(dir) == 3:
        #     for i in offsets:
        #         loc = (start[0] + i[0], start[1] + i[1])
        #         if loc in self.nodes:
        #             dir = self.nodes[loc].dir
        #             break

        # return dir

        closet = None
        min = 999999999999999
        for i in offsets:
            loc = (start[0] + i[0], start[1] + i[1])
            dist = util.length((pos[0] - loc[0], pos[1] - loc[1]))
            if loc in self.nodes and dist < min:
                closest = self.nodes[loc]
                min = dist

        return closest.dir

    def render(self, surface, offset=(0, 0), scale=1):
        # render every node
        for key in self.nodes:
            self.nodes[key].render(surface, offset, scale)