# group.py

import util
from problem import ContinuousNavigation, Node
from area import Area
import math


class PathReservationSystem:
    """
    Manages reserved locations on a time-discretized grid to prevent agent collisions.
    """

    def __init__(self, step_density=5.0, time_step_scale=1.0):
        # Maps a (grid_x, grid_y, time_step) tuple to the ID of the agent that reserved it.
        # Grid cells are discrete, time is discrete.
        self.reservations = {}
        self.step_density = step_density
        self.time_step_scale = time_step_scale
        self.agent_path_reservations = {}  # Stores all reservations for a specific agent ID

    def get_grid_pos(self, pos: tuple[float, float]) -> tuple[int, int]:
        """Converts continuous position to discrete grid coordinates."""
        grid_x = int(pos[0] // self.step_density)
        grid_y = int(pos[1] // self.step_density)
        return (grid_x, grid_y)

    def reserve_path(self, agent_id: int, path_moves: list[tuple], start_pos: tuple[float, float]):
        """
        Reserves space-time blocks for an entire calculated path.

        Args:
            agent_id (int): Unique ID of the agent making the reservation.
            path_moves (list[tuple]): Sequence of (dx, dy) moves from an A* search.
            start_pos (tuple): Continuous starting position (x, y).
        """
        current_pos = start_pos
        time_step = 0

        # Clear old reservations for this agent
        if agent_id in self.agent_path_reservations:
            for reservation_key in self.agent_path_reservations[agent_id]:
                if reservation_key in self.reservations:
                    del self.reservations[reservation_key]

        new_reservations = []

        # Start by reserving the current cell at time 0
        grid_pos = self.get_grid_pos(current_pos)
        key = (grid_pos[0], grid_pos[1], time_step)

        # Only reserve if it's not currently reserved by another agent
        if key not in self.reservations or self.reservations[key] == agent_id:
            self.reservations[key] = agent_id
            new_reservations.append(key)

        for move in path_moves:
            time_step += 1

            # Estimate the new continuous position
            move_length = util.length(move)
            agent_speed = 0.5  # Assuming standard agent speed from agent.py

            # Calculate the full position for the next move
            current_pos = (current_pos[0] + (move[0] / move_length) * agent_speed,
                           current_pos[1] + (move[1] / move_length) * agent_speed)

            grid_pos = self.get_grid_pos(current_pos)
            key = (grid_pos[0], grid_pos[1], time_step)

            # Reserve the new cell at the new time step
            if key not in self.reservations or self.reservations[key] == agent_id:
                self.reservations[key] = agent_id
                new_reservations.append(key)

        self.agent_path_reservations[agent_id] = new_reservations

    def is_reserved(self, pos: tuple[float, float], time_step: int, current_agent_id: int) -> bool:
        """Checks if a grid cell at a specific time step is reserved by another agent."""
        grid_pos = self.get_grid_pos(pos)
        key = (grid_pos[0], grid_pos[1], time_step)

        # A cell is reserved if it's in the dictionary and reserved by someone else
        return key in self.reservations and self.reservations[key] != current_agent_id


# Global instance of the reservation system
ReservationSystem = PathReservationSystem()