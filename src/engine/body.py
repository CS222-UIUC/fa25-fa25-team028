import numpy as np

# mass, position, velocity and force
class Body:
    def __init__(self, mass, position, velocity,
                 radius=0.5, restitution=1.0, is_static=False):
        self.mass = float(mass)
        self.position = np.array(position, dtype=float)
        self.velocity = np.array(velocity, dtype=float)
        self.force = np.zeros(2, dtype=float)

        self.radius = float(radius)
        self.restitution = float(restitution)
        self.is_static = bool(is_static)


        self.sleeping = False     
        self.sleep_time = 0.0     

    def apply_force(self, force):
        self.force += np.array(force, dtype=float)

    def clear_force(self):

        self.force[:] = 0.0

    def kinetic_energy(self):
        velocity_magnitude_squared = np.dot(self.velocity, self.velocity)
        return 0.5 * self.mass * velocity_magnitude_squared

    def momentum(self):
        return self.mass * self.velocity

    def gravitational_potential_energy(self, g=9.81, reference_height=0.0):
        height = float(self.position[1])
        return self.mass * float(g) * (height - float(reference_height))

    @property
    def inverse_mass(self):
        """mass inverter"""
        if self.is_static or not np.isfinite(self.mass):
            return 0.0
        return 1.0 / self.mass

        
    def put_to_sleep(self):
        """sleep"""
        self.sleeping = True
        self.sleep_time = 0.0
        self.velocity[:] = 0.0
        self.clear_force()

    def wake(self):
        """wake up"""
        self.sleeping = False
        self.sleep_time = 0.0