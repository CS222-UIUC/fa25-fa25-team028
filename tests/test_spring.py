import pytest
import numpy as np
from engine import Body, Spring


class TestSpring:
    def test_initialization(self):
        """Testing that the spring initializes with correct initial values"""
        body1 = Body(mass=1.0, position=[0, 0], velocity=[0, 0])
        body2 = Body(mass=1.0, position=[1, 0], velocity=[0, 0])
        spring = Spring(body1, body2, k=10.0, rest_length=0.5)

        assert spring.k == 10.0
        assert spring.rest_length == 0.5
        assert spring.body1 == body1
        assert spring.body2 == body2

    def test_force_at_rest(self):
        """Testing that the spring has no force when at rest"""
        body1 = Body(mass=1.0, position=[0, 0], velocity=[0, 0])
        body2 = Body(mass=1.0, position=[1, 0], velocity=[0, 0])
        spring = Spring(body1, body2, k=10.0, rest_length=1.0)
        spring.apply_forces()

        assert np.allclose(body1.force, [0, 0])
        assert np.allclose(body2.force, [0, 0])

    def test_force_when_stretched(self):
        """Testing that the springs have inward forces when stretched"""
        body1 = Body(mass=1.0, position=[0, 0], velocity=[0, 0])
        body2 = Body(mass=1.0, position=[2, 0], velocity=[0, 0])
        spring = Spring(body1, body2, k=10.0, rest_length=1.0)
        spring.apply_forces()

        assert body1.force[0] > 0
        assert body2.force[0] < 0
        assert np.isclose(body1.force[0], -body2.force[0])

    def test_force_when_compressed(self):
        """Testing that the springs have outward forces when test_force_when_compressed"""
        body1 = Body(mass=1.0, position=[0, 0], velocity=[0, 0])
        body2 = Body(mass=1.0, position=[0.5, 0], velocity=[0, 0])
        spring = Spring(body1, body2, k=10.0, rest_length=1.0)
        spring.apply_forces()

        assert body1.force[0] < 0
        assert body2.force[0] > 0
        assert np.isclose(body1.force[0], -body2.force[0])

    def test_force_with_zero_length(self):
        """Testing that the zero length edge case is caught"""
        body1 = Body(mass=1.0, position=[0, 0], velocity=[0, 0])
        body2 = Body(mass=1.0, position=[0, 0], velocity=[0, 0])
        spring = Spring(body1, body2, k=10.0, rest_length=1.0)
        spring.apply_forces()

        assert np.allclose(body1.force, [0, 0])
        assert np.allclose(body2.force, [0, 0])

    def test_damping_force(self):
        """Testing that the damping force opposes the force of body 2"""
        body1 = Body(mass=1.0, position=[0, 0], velocity=[0, 0])
        body2 = Body(mass=1.0, position=[1, 0], velocity=[1, 0])
        spring = Spring(body1, body2, k=10.0, rest_length=1.0)
        damping_force = spring.damping_force()

        assert damping_force[0] < 0
        assert damping_force[1] == 0
