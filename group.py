"""
reservation.py

Implements a Windowed Hierarchical Cooperative A* (WHCA*) reservation system.

This module maintains a time-windowed reservation table that prevents
multi-agent collisions by reserving future grid cells. It is designed
to be fully compatible with the CoordinatedAgent implementation.
"""

import util

class WHCAReservationSystem:
    """
    Windowed Hierarchical Cooperative A* Reservation Table
    Fully compatible with your current CoordinatedAgent implementation.
    """

    def __init__(self, grid_size=2.0, window=5):
        self.grid_size = grid_size
        self.window = window

        # (grid_x, grid_y, time) → agent_id
        self.res_table = {}

        # agent_id → list of reservation keys
        self.agent_last_res = {}


    def to_grid(self, pos):
        return (
            int(pos[0] // self.grid_size),
            int(pos[1] // self.grid_size)
        )

    # --------------------------------------------------------------
    def reserve_path(self, agent_id, moves, start_pos):
        # Clear previous reservations by this agent
        if agent_id in self.agent_last_res:
            for key in self.agent_last_res[agent_id]:
                if key in self.res_table and self.res_table[key] == agent_id:
                    del self.res_table[key]

        new_keys = []
        cx, cy = start_pos
        t = 0

        # Reserve starting cell at time 0
        gx, gy = self.to_grid((cx, cy))
        key = (gx, gy, t % self.window)
        self.res_table[key] = agent_id
        new_keys.append(key)

        # Reserve along the A* path for the next "window" timesteps
        for mv in moves:
            t += 1
            length = util.length(mv)
            if length == 0:
                continue

            cx += (mv[0] / length) * 0.5
            cy += (mv[1] / length) * 0.5

            gx, gy = self.to_grid((cx, cy))
            key = (gx, gy, t % self.window)

            # Priority rule: lower agent_id wins WHCA reservations
            if key not in self.res_table or self.res_table[key] > agent_id:
                self.res_table[key] = agent_id

            new_keys.append(key)

            if t >= self.window:
                break

        self.agent_last_res[agent_id] = new_keys

    # --------------------------------------------------------------
    def is_reserved(self, pos, time_step, agent_id):
        """
        Returns True if this position is blocked by another agent in the WHCA* table.
        Uses relaxed collision testing to reduce deadlocks.
        """
        gx, gy = self.to_grid(pos)
        key = (gx, gy, time_step % self.window)

        # If no one reserved it, it's free
        if key not in self.res_table:
            return False

        other = self.res_table[key]

        # If we reserved it ourselves, it's free
        if other == agent_id:
            return False

        # Soft collision test — allow near misses if safe
        cell_center_x = gx * self.grid_size + self.grid_size / 2
        cell_center_y = gy * self.grid_size + self.grid_size / 2

        dist = util.length((cell_center_x - pos[0], cell_center_y - pos[1]))

        # Allow close-but-safe movements
        if dist > 0.7:
            return False

        # Otherwise, it's treated as a conflict
        return True


# Global WHCA* reservation system instance used by CoordinatedAgent
ReservationSystem = WHCAReservationSystem()