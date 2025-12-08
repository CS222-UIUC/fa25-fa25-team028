from .collisions import CollisionSystem
import numpy as np
from .integrator import euler_step
SLEEP_SPEED = 0.1    
SLEEP_TIME = 0.5      
AIR_RESISTANCE = 0.98

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
        for spring in self.springs:
            spring.apply_forces()

        for body in self.bodies:
            if getattr(body, "is_static", False) or body.mass >= 1e9:
                body.clear_force()
                continue
            if body.sleeping:
                f2 = float(np.dot(body.force, body.force))
                if f2 > 1e-6:
                    body.wake()
                else:
                    body.clear_force()
                    continue
            euler_step(body, self.dt)
            body.velocity *= AIR_RESISTANCE
        if self.collision_system is not None:
            self.collision_system.step(self.bodies)

        v2_threshold = SLEEP_SPEED * SLEEP_SPEED

        for body in self.bodies:
            if getattr(body, "is_static", False) or body.mass >= 1e9:
                continue

            speed2 = float(np.dot(body.velocity, body.velocity))

            if speed2 < v2_threshold:
                body.sleep_time += self.dt
                if body.sleep_time >= SLEEP_TIME:
                    body.put_to_sleep()
            else:
                body.wake()