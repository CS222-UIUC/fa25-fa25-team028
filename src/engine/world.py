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

        for body in self.bodies:
            euler_step(body, self.dt)
