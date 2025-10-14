import numpy as np


class Spring:
    def __init__(self, body1, body2, k, rest_length):
        self.body1 = body1
        self.body2 = body2
        self.k = k
        self.rest_length = rest_length
        # need to create way for user to control damping ratio/coefficient
        self.damping = 0.1

    def apply_forces(self):
        # Hooke's law: F = -k * x
        displacement = self.body2.position - self.body1.position
        length = np.linalg.norm(displacement)
        if length == 0:
            return
        force_magnitude = self.k * (length - self.rest_length)
        direction = displacement / length  # unit vector
        total_force = direction * force_magnitude + self.damping_force()
        self.body1.force += total_force
        self.body2.force += -total_force

    def damping_force(self):
        # Damping Force= -damping coefficient * relative velocity
        relative_velocity = self.body2.velocity - self.body1.velocity
        return -self.damping * relative_velocity
