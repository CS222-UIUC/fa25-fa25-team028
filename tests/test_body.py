from engine.body import Body
from engine.integrator import euler_step
from engine.world import World
import numpy as np


def test_body_init():
    b = Body(mass=2.5, position=[1, -2], velocity=[0.5, 1.0])
    assert b.mass == 2.5
    assert isinstance(b.position, np.ndarray) and isinstance(b.velocity, np.ndarray)
    np.testing.assert_allclose(b.position, [1.0, -2.0])
    np.testing.assert_allclose(b.velocity, [0.5, 1.0])
    np.testing.assert_allclose(b.force, [0.0, 0.0])


def test_apply_force_accumulates():
    b = Body(1.0, [0, 0], [0, 0])
    b.apply_force([1, 2])
    b.apply_force([-0.5, 0.5])
    np.testing.assert_allclose(b.force, [0.5, 2.5])


def test_clear_force_resets_to_zero():
    b = Body(1.0, [0, 0], [0, 0])
    b.apply_force([3, -4])
    b.clear_force()
    np.testing.assert_allclose(b.force, [0.0, 0.0])


def test_euler_step_updates_and_clears_force():
    b = Body(2.0, [0, 0], [0, 0])
    b.apply_force([2, -4])

    dt = 0.1
    euler_step(b, dt)

    # v_new = v + a*dt = [0,0] + [1,-2]*0.1 = [0.1, -0.2]
    # p_new = p + v_new*dt = [0,0] + [0.1,-0.2]*0.1 = [0.01, -0.02]
    np.testing.assert_allclose(b.velocity, [0.1, -0.2], rtol=0, atol=1e-12)
    np.testing.assert_allclose(b.position, [0.01, -0.02], rtol=0, atol=1e-12)
    np.testing.assert_allclose(b.force, [0.0, 0.0])


def test_world_step_one_body_no_springs():
    b = Body(1.0, [0, 0], [0, 0])
    w = World(dt=0.2)
    w.add_body(b)

    # F = [5, 0] => a = [5, 0]; dt=0.2
    b.apply_force([5, 0])
    w.step()

    # v = [1, 0]; p = [0.2, 0]
    np.testing.assert_allclose(b.velocity, [1.0, 0.0])
    np.testing.assert_allclose(b.position, [0.2, 0.0])
    np.testing.assert_allclose(b.force, [0.0, 0.0])  # emptied !


def test_two_euler_steps_match_manual():
    b = Body(2.0, [1.0, 1.0], [0.0, 0.0])
    dt = 0.05

    #euler step1
    b.apply_force([4, 2])
    euler_step(b, dt)
    #euler step2
    b.apply_force([4, 2])
    euler_step(b, dt)

    # step1: v1 = 0 + [2,1]*0.05 = [0.1, 0.05]; p1 = [1,1] + v1*0.05 = [1.005, 1.0025]
    # step2: v2 = v1 + [2,1]*0.05 = [0.2, 0.1];  p2 = p1 + v2*0.05 = [1.015, 1.0075]
    np.testing.assert_allclose(b.velocity, [0.2, 0.1], rtol=0, atol=1e-12)
    np.testing.assert_allclose(b.position, [1.015, 1.0075], rtol=0, atol=1e-12)