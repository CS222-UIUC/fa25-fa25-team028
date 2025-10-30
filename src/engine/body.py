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

    def kinetic_energy(self):
        velocity_magnitude_squared = np.dot(self.velocity, self.velocity)
        return 0.5 * self.mass * velocity_magnitude_squared

    def apply_gravity(self, g=9.8):
        gravity_force = np.array([0, -self.mass * g])
        self.apply_force(gravity_force)

    def momentum(self):
        return self.mass * self.velocity
    