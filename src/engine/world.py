from .collisions import CollisionSystem


class World:
    def __init__(self, dt=0.1, collision_system=None):
        self.dt = dt
        self.bodies = []
        self.springs = []
        self.collision_system = collision_system or CollisionSystem()

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
            euler_step(body, self.dt)

        if self.collision_system is not None:
            self.collision_system.step(self.bodies)
