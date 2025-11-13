import numpy as np
import pytest

from engine.rigidbody2d import PolygonShape, RigidBody2D


@pytest.fixture
def rect_shape():
    """
    Rectangle 2x1 centered at origin (local vertices, CCW).
    Area = 2 * 1 = 2
    """
    return PolygonShape(np.array([
        [-1.0, -0.5],
        [ 1.0, -0.5],
        [ 1.0,  0.5],
        [-1.0,  0.5],
    ], dtype=float))


# -------- Shape tests --------

def test_polygonshape_requires_3_vertices():
    with pytest.raises(ValueError):
        PolygonShape(np.array([[0.0, 0.0],
                               [1.0, 0.0]], dtype=float))

def test_polygonshape_area_and_centroid(rect_shape):
    # area=2, centroid≈(0,0)
    assert pytest.approx(rect_shape.area, rel=0, abs=1e-12) == 2.0
    np.testing.assert_allclose(rect_shape.local_centroid, [0.0, 0.0], atol=1e-12)


# rigidbody2d initialization

def test_rigidbody2d_mass_vs_density(rect_shape):
    # Using mass directly
    rb_m = RigidBody2D(rect_shape, mass=2.5, position=(0, 0))
    assert rb_m.mass == pytest.approx(2.5)

    # Using density (mass = density * area), area=2 -> mass=6.0
    rb_d = RigidBody2D(rect_shape, density=3.0, position=(0, 0))
    assert rb_d.mass == pytest.approx(6.0)

def test_rigidbody2d_mass_density_exclusivity(rect_shape):
    # Neither provided
    with pytest.raises(ValueError):
        RigidBody2D(rect_shape, position=(0, 0))
    # Both provided
    with pytest.raises(ValueError):
        RigidBody2D(rect_shape, mass=1.0, density=1.0, position=(0, 0))



def test_integrate_updates_linear_state(rect_shape):
    rb = RigidBody2D(rect_shape, mass=2.0, position=(0, 0), velocity=(0, 0))
    # net acceleration -> [1, -2]
    rb.apply_force([2.0, -4.0])
    dt = 0.1
    rb.integrate(dt)

    # v_new = [0,0] + a*dt = [0.1, -0.2]
    # p_new = [0,0] + v_new*dt = [0.01, -0.02]
    np.testing.assert_allclose(rb.velocity, [0.1, -0.2], atol=1e-12)
    np.testing.assert_allclose(rb.position, [0.01, -0.02], atol=1e-12)

    # clear force and torque
    np.testing.assert_allclose(rb.force, [0.0, 0.0], atol=1e-12)
    assert rb.torque == pytest.approx(0.0, abs=1e-12)



def test_apply_force_offcenter_produces_torque(rect_shape):
    rb = RigidBody2D(rect_shape, mass=2.0, position=(0, 0), velocity=(0, 0), angle=0.0)
    # apply force at a point to test torque!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    f = np.array([10.0, 0.0])
    world_point = np.array([0.0, 0.5])  # r = (0, 0.5); cross(r,f) = -5 < 0
    rb.apply_force(f, world_point=world_point)
    dt = 0.05
    rb.integrate(dt)

    # angular velocity follow torque
    assert rb.angular_velocity < 0.0


def test_support_point_and_projection(rect_shape):
    rb = RigidBody2D(rect_shape, mass=2.0, position=(0.0, 0.0), angle=0.0)

    # direction: 1
    sp = rb.support_point([1.0, 0.0])
    assert sp[0] == pytest.approx(1.0, abs=1e-12)

    # projection -> [-1, 1]
    pmin, pmax = rb.project_onto_axis([1.0, 0.0])
    assert pmin == pytest.approx(-1.0, abs=1e-10)
    assert pmax == pytest.approx( 1.0, abs=1e-10)

# Rotation test
def test_aabb_rotated(rect_shape):
    rb = RigidBody2D(rect_shape, mass=1.0, position=(0.0, 0.0), angle=np.pi/2)
    (mn, mx) = rb.aabb()
    # 2x1 rectangle rotated about origin
    np.testing.assert_allclose(mn, [-0.5, -1.0], atol=1e-12)
    np.testing.assert_allclose(mx, [ 0.5,  1.0], atol=1e-12)