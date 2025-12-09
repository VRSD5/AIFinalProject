import search


class Pathfinder:
    def __init__(self, method, use_hierarchical=False):
        self.method = method
        self.use_hierarchical = use_hierarchical
        self.waiting = set()
        self.queue = []
        self.complete = {}
        self.goal_groups = {}
        self.agent_paths = {}  # NEW: Cache the last successful path for all agents

    def queue_path(self, index, problem):
        self.queue.append((index, problem))
        self.waiting.add(index)

        # Group agents by goal location
        goal_pos = problem.goal_state.get_center()
        if goal_pos not in self.goal_groups:
            self.goal_groups[goal_pos] = []
        self.goal_groups[goal_pos].append(index)

    def get_priority_paths(self, current_agent_index):
        """NEW METHOD: Collects paths for all agents with higher priority (lower index)."""
        priority_paths = {}
        for idx, path in self.agent_paths.items():
            # If the other agent's index (idx) is lower, it has higher priority
            if idx < current_agent_index:
                priority_paths[idx] = path
        return priority_paths

    def pop_queue(self):
        """Process one pathfinding request"""
        if not self.queue:
            return

        unit = self.queue.pop(0)
        index, prob = unit

        # NEW: Collect and apply constraints from higher-priority agents
        higher_priority_paths = self.get_priority_paths(index)

        # This calls the NEW method added to ContinuousNavigation
        prob.add_constraints(higher_priority_paths)

        # --- PATH CALCULATION ---
        if self.use_hierarchical:
            # We must call the prioritized version of the search function
            path = search.hierarchical_astar_prioritized(prob)
        else:
            # We would use a prioritized A* if not using HA*
            path = self.method(prob)
            # ------------------------

        self.complete[index] = path
        self.agent_paths[index] = path  # NEW: Store the newly computed path
        if index in self.waiting:
            self.waiting.remove(index)

    def process_group(self, count=10):
        """Process multiple pathfinding requests in batch (adapted for priority)"""
        processed = 0

        goals_to_process = list(self.goal_groups.keys())
        goal_index = 0

        while processed < count and self.queue:
            if goals_to_process:
                current_goal = goals_to_process[goal_index % len(goals_to_process)]

                for i, (idx, prob) in enumerate(self.queue):
                    if prob.goal_state.get_center() == current_goal:
                        # NEW: Collect and apply constraints from higher-priority agents
                        higher_priority_paths = self.get_priority_paths(idx)
                        prob.add_constraints(higher_priority_paths)

                        # Process this request
                        if self.use_hierarchical:
                            path = search.hierarchical_astar_prioritized(prob)
                        else:
                            path = self.method(prob)

                        self.complete[idx] = path
                        self.agent_paths[idx] = path
                        if idx in self.waiting:
                            self.waiting.remove(idx)
                        self.queue.pop(i)
                        processed += 1
                        break

                goal_index += 1
            else:
                self.pop_queue()
                processed += 1

        self.goal_groups = {k: v for k, v in self.goal_groups.items() if v}