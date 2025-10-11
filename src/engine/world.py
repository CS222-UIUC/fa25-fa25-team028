class World:
    def __init__(self, dt=0.1):
        self.dt = dt
        self.bodies = []

    def add_body(self, body):
        self.bodies.append(body)

    def step(self):
        from .integrator import euler_step
        for body in self.bodies:
            euler_step(body, self.dt)