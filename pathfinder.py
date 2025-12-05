


class Pathfinder:
    pather = None


    
    def __init__(self, method):
        self.method = method
        self.waiting = set()
        self.queue = []
        self.complete = {}

    def queue_path(self, index, problem):
        self.queue.append((index, problem))
        self.waiting.add(index)
    
    def pop_queue(self):
        unit = self.queue.pop(0)
        path = self.method(unit[1])
        self.complete[unit[0]] = path

