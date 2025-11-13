class World:
    def __init__(self, dt=0.1):
        self.dt = dt
        self.bodies = []
        self.springs = []

    def add_body(self, body):
        self.bodies.append(body)

    def add_spring(self, spring):
        self.springs.append(spring)

    def step(self):
        from .integrator import euler_step

        for spring in self.springs:
            spring.apply_forces()

        # Add global velocity damping (air resistance)
        AIR_RESISTANCE = 0.98  # Multiply velocity by this each step

        for body in self.bodies:
            if body.mass < 1e9:
                euler_step(body, self.dt)
                body.velocity *= AIR_RESISTANCE  # Dampen all motion
            else:
                body.clear_force()