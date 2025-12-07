import numpy as np


class CollisionSystem:
    def __init__(self, penetration_slop=1e-3, floor_y=None, floor_enabled=False,
                 wall_left=None, wall_right=None, walls_enabled=False, bounce_damping=0.7):
        self.penetration_slop = penetration_slop
        self.floor_y = floor_y if floor_y is not None else -5.0
        self.floor_enabled = floor_enabled
        self.wall_left = wall_left
        self.wall_right = wall_right
        self.walls_enabled = walls_enabled
        self.bounce_damping = bounce_damping

    def step(self, bodies):
        for i, body_a in enumerate(bodies):
            for j in range(i + 1, len(bodies)):
                body_b = bodies[j]
                info = self._circle_circle(body_a, body_b)
                if info is None:
                    continue
                normal, penetration = info
                self._apply_impulse(body_a, body_b, normal)
                self._separate(body_a, body_b, normal, penetration)
            for body in bodies:
                # Skip fixed bodies
                if hasattr(body, 'mass') and body.mass >= 1e9:
                    continue
                # Skip static bodies
                if hasattr(body, 'is_static') and body.is_static:
                    continue

                self._check_floor_collision(body)
                self._check_wall_collisions(body)

    def _check_floor_collision(self, body):
        """Check and handle floor collision"""
        if not self.floor_enabled:
            return

        radius = getattr(body, 'radius', 0.2)

        if body.position[1] - radius < self.floor_y:
            body.position[1] = self.floor_y + radius
            if body.velocity[1] < 0:  # Only if moving downward
                body.velocity[1] = -body.velocity[1] * self.bounce_damping

    def _check_wall_collisions(self, body):
        """Check and handle wall collisions"""
        if not self.walls_enabled:
            return

        radius = getattr(body, 'radius', 0.2)

        # Left wall
        if self.wall_left is not None and body.position[0] - radius < self.wall_left:
            body.position[0] = self.wall_left + radius
            if body.velocity[0] < 0:
                body.velocity[0] = -body.velocity[0] * self.bounce_damping

        # Right wall
        if self.wall_right is not None and body.position[0] + radius > self.wall_right:
            body.position[0] = self.wall_right - radius
            if body.velocity[0] > 0:
                body.velocity[0] = -body.velocity[0] * self.bounce_damping

    def _circle_circle(self, body_a, body_b):
        radius_sum = body_a.radius + body_b.radius
        delta = body_b.position - body_a.position
        distance_sq = np.dot(delta, delta)
        if distance_sq >= radius_sum ** 2:
            return None
        distance = np.sqrt(distance_sq)
        if distance <= 1e-8:
            normal = np.array([1.0, 0.0])
            penetration = radius_sum
        else:
            normal = delta / distance
            penetration = radius_sum - distance
        return normal, penetration

    def _apply_impulse(self, body_a, body_b, normal):
        inv_mass_a = body_a.inverse_mass
        inv_mass_b = body_b.inverse_mass
        total_inv_mass = inv_mass_a + inv_mass_b
        if total_inv_mass == 0.0:
            return

        relative_velocity = body_b.velocity - body_a.velocity
        vel_along_normal = np.dot(relative_velocity, normal)
        if vel_along_normal > 0.0:
            return

        restitution = min(body_a.restitution, body_b.restitution)
        impulse_mag = -(1.0 + restitution) * vel_along_normal
        impulse_mag /= total_inv_mass
        impulse = impulse_mag * normal

        if not body_a.is_static:
            body_a.velocity -= impulse * inv_mass_a
        if not body_b.is_static:
            body_b.velocity += impulse * inv_mass_b

    def _separate(self, body_a, body_b, normal, penetration):
        inv_mass_a = body_a.inverse_mass
        inv_mass_b = body_b.inverse_mass
        total_inv_mass = inv_mass_a + inv_mass_b
        if total_inv_mass == 0.0:
            return
        correction = max(penetration - self.penetration_slop, 0.0)
        if correction == 0.0:
            return
        correction /= total_inv_mass
        if not body_a.is_static:
            body_a.position -= normal * correction * inv_mass_a
        if not body_b.is_static:
            body_b.position += normal * correction * inv_mass_b

