"""
pathfinder.py

Asynchronous / deferred pathfinding helper.

This module provides a lightweight queue-based wrapper around a
pathfinding function (e.g., A*). It allows agents to request paths
without blocking the main simulation loop, enabling staggered or
batched path computation.
"""

class Pathfinder:
    """
    Deferred pathfinding queue manager.

    Agents submit pathfinding problems which are processed incrementally.
    Completed paths are stored and can be retrieved by agent index.

    Attributes:
        method (callable):
            Pathfinding function that accepts a problem instance and
            returns a list of movement vectors.
        waiting (set[int]):
            Agent indices currently waiting for a path.
        queue (list[tuple[int, object]]):
            FIFO queue of (agent_index, problem) requests.
        complete (dict[int, list]):
            Completed paths indexed by agent id.
    """


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

