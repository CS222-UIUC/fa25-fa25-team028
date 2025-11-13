import numpy as np


class CollisionSystem:
    def __init__(self, penetration_slop=1e-3):
        self.penetration_slop = penetration_slop

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

