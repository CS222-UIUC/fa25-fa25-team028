import pytest
from engine.utils import create_chain
from engine.body import Body
from engine.spring import Spring


class TestCreateChain:

    def test_create_chain_correct_counts(self):
        """Test that chain creates correct number of bodies and springs"""
        bodies, springs = create_chain(5, [0, 0], 1.0, 1.0, 10.0, 1.0)

        assert len(bodies) == 5
        assert len(springs) == 4

    def test_create_chain_horizontal_positions(self):
        """Test that horizontal chain positions bodies correctly"""
        bodies, springs = create_chain(3, [0, 0], 2.0, 1.0, 10.0, 1.0, orientation='horizontal')

        assert bodies[0].position[0] == 0.0
        assert bodies[0].position[1] == 0.0

        assert bodies[1].position[0] == 2.0
        assert bodies[1].position[1] == 0.0

        assert bodies[2].position[0] == 4.0
        assert bodies[2].position[1] == 0.0

    def test_create_chain_vertical_positions(self):
        """Test that vertical chain positions bodies correctly"""
        bodies, springs = create_chain(3, [10, 0], 2.0, 1.0, 10.0, 1.0, orientation='vertical')

        assert bodies[0].position[0] == 10.0
        assert bodies[0].position[1] == 0.0

        assert bodies[1].position[0] == 10.0
        assert bodies[1].position[1] == 2.0

        assert bodies[2].position[0] == 10.0
        assert bodies[2].position[1] == 4.0

    def test_create_chain_body_properties(self):
        """Test that bodies have correct mass"""
        bodies, springs = create_chain(3, [0, 0], 1.0, 2.5, 10.0, 1.0)

        for body in bodies:
            assert body.mass == 2.5
            assert body.velocity[0] == 0.0
            assert body.velocity[1] == 0.0

    def test_create_chain_spring_properties(self):
        """Test that springs have correct properties"""
        bodies, springs = create_chain(3, [0, 0], 1.0, 1.0, 15.0, 0.8, damping=0.05)

        assert len(springs) == 2
        for spring in springs:
            assert spring.k == 15.0
            assert spring.rest_length == 0.8
            assert spring.damping == 0.05

    def test_create_chain_springs_connect_correctly(self):
        """Test that springs connect consecutive bodies"""
        bodies, springs = create_chain(4, [0, 0], 1.0, 1.0, 10.0, 1.0)

        # First spring connects body 0 and 1
        assert springs[0].body1 == bodies[0]
        assert springs[0].body2 == bodies[1]

        # Second spring connects body 1 and 2
        assert springs[1].body1 == bodies[1]
        assert springs[1].body2 == bodies[2]

        # Third spring connects body 2 and 3
        assert springs[2].body1 == bodies[2]
        assert springs[2].body2 == bodies[3]

    def test_create_chain_invalid_orientation(self):
        """Test that invalid orientation raises error"""
        with pytest.raises(ValueError):
            create_chain(3, [0, 0], 1.0, 1.0, 10.0, 1.0, orientation='diagonal')