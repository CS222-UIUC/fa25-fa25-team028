import numpy as np

from engine.body import Body
from engine.collisions import CollisionSystem


def test_circle_circle_elastic_collision():
    a = Body(mass=1.0, position=[-0.25, 0.0], velocity=[1.0, 0.0], radius=0.5)
    b = Body(mass=1.0, position=[0.25, 0.0], velocity=[-1.0, 0.0], radius=0.5)
    system = CollisionSystem()

    system.step([a, b])

    np.testing.assert_allclose(a.velocity, [-1.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(b.velocity, [1.0, 0.0], atol=1e-12)


def test_circle_hits_static_body_stops():
    falling = Body(
        mass=1.0,
        position=[0.0, 0.6],
        velocity=[0.0, -2.0],
        radius=0.5,
        restitution=0.0,
    )
    static = Body(
        mass=1.0,
        position=[0.0, 0.0],
        velocity=[0.0, 0.0],
        radius=0.5,
        restitution=0.0,
        is_static=True,
    )
    system = CollisionSystem()

    system.step([falling, static])

    np.testing.assert_allclose(falling.velocity, [0.0, 0.0], atol=1e-12)
    assert falling.position[1] >= static.position[1] + falling.radius + static.radius - 1e-6

