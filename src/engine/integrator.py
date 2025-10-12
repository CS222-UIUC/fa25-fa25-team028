def euler_step(body, dt):
    a = body.force / body.mass
    body.velocity += a * dt
    body.position += body.velocity * dt
    body.clear_force()