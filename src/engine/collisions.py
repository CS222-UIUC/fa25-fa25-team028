import numpy as np


class CollisionSystem:
    def __init__(self, penetration_slop=1e-3, floor_y=None, floor_enabled=False,
                 wall_left=None, wall_right=None, walls_enabled=False,
                 bounce_damping=0.7, friction=0.5):
        self.penetration_slop = penetration_slop
        self.floor_y = floor_y if floor_y is not None else -5.0
        self.floor_enabled = floor_enabled
        self.wall_left = wall_left
        self.wall_right = wall_right
        self.walls_enabled = walls_enabled
        self.bounce_damping = bounce_damping
        self.friction = friction

    def step(self, bodies):
        for i, body_a in enumerate(bodies):
            for j in range(i + 1, len(bodies)):
                body_b = bodies[j]

                info = self._aabb_aabb(body_a, body_b)
                if info is None:
                    continue

                normal, penetration = info
                self._apply_impulse(body_a, body_b, normal)
                self._separate(body_a, body_b, normal, penetration)

        for body in bodies:
            # Skip static or infinite-mass bodies
            if getattr(body, "mass", 1) >= 1e9:
                continue
            if getattr(body, "is_static", False):
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
        jn = -(1.0 + restitution) * vel_along_normal
        jn /= (total_inv_mass + 1e-12)
        impulse_n = jn * normal

        if not body_a.is_static:
            body_a.velocity -= impulse_n * inv_mass_a
        if not body_b.is_static:
            body_b.velocity += impulse_n * inv_mass_b
        relative_velocity = body_b.velocity - body_a.velocity

        vel_along_normal = np.dot(relative_velocity, normal)
        tangent = relative_velocity - vel_along_normal * normal
        t_len = np.linalg.norm(tangent)
        if t_len < 1e-6:
            return  

        tangent /= t_len

        vt = np.dot(relative_velocity, tangent)
        jt = -vt / (total_inv_mass + 1e-12)

        mu = self.friction
        max_jt = mu * jn
        jt = np.clip(jt, -max_jt, max_jt)

        impulse_t = jt * tangent

        if not body_a.is_static:
            body_a.velocity -= impulse_t * inv_mass_a
        if not body_b.is_static:
            body_b.velocity += impulse_t * inv_mass_b

        relative_velocity = body_b.velocity - body_a.velocity

        vel_along_normal = np.dot(relative_velocity, normal)
        tangent = relative_velocity - vel_along_normal * normal
        t_len = np.linalg.norm(tangent)
        if t_len < 1e-6:
            pass
        else:
            tangent /= t_len

            vt = np.dot(relative_velocity, tangent)
            jt = -vt / (total_inv_mass + 1e-12)

            mu = self.friction
            max_jt = mu * jn
            jt = np.clip(jt, -max_jt, max_jt)

            impulse_t = jt * tangent

            if not body_a.is_static:
                body_a.velocity -= impulse_t * inv_mass_a
            if not body_b.is_static:
                body_b.velocity += impulse_t * inv_mass_b

        impulse_mag = abs(jn)  
        WAKE_THRESHOLD = 1e-4

        if impulse_mag > WAKE_THRESHOLD:
            if hasattr(body_a, "wake"):
                body_a.wake()
            if hasattr(body_b, "wake"):
                body_b.wake()
    def _separate(self, body_a, body_b, normal, penetration):
        """separate"""
        if penetration <= 0.0:
            return

        total_inv_mass = body_a.inverse_mass + body_b.inverse_mass
        if total_inv_mass == 0.0:
            return

        correction_mag = max(penetration - self.penetration_slop, 0.0) / total_inv_mass
        correction = correction_mag * normal

        if not body_a.is_static:
            body_a.position -= correction * body_a.inverse_mass
        if not body_b.is_static:
            body_b.position += correction * body_b.inverse_mass

    def _aabb_aabb(self, body_a, body_b):
        """
        AABB collision
        """
        ha = getattr(body_a, "half_extents",
                     np.array([body_a.radius, body_a.radius], dtype=float))
        hb = getattr(body_b, "half_extents",
                     np.array([body_b.radius, body_b.radius], dtype=float))

        delta = body_b.position - body_a.position
        dx = delta[0]
        dy = delta[1]

        overlap_x = ha[0] + hb[0] - abs(dx)
        overlap_y = ha[1] + hb[1] - abs(dy)

        if overlap_x <= 0.0 or overlap_y <= 0.0:
            return None

        if overlap_x < overlap_y:
            if dx >= 0.0:
                normal = np.array([1.0, 0.0])
            else:
                normal = np.array([-1.0, 0.0])
            penetration = overlap_x
        else:
            if dy >= 0.0:
                normal = np.array([0.0, 1.0])
            else:
                normal = np.array([0.0, -1.0])
            penetration = overlap_y

        return normal, penetration
