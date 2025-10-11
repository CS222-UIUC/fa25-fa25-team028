import numpy as np

# mass, position, velocity and force

class Body:
    def __init__(self, mass, position, velocity):
        self.mass = mass
        self.position = np.array(position, dtype=float)
        self.velocity = np.array(velocity, dtype=float)
        self.force = np.zeros(2, dtype=float)

    def apply_force(self, force):
        self.force += np.array(force, dtype=float)

    def clear_force(self):
        self.force = np.zeros(2, dtype=float)