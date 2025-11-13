from .body import Body
from .spring import Spring


def create_chain(num_bodies, start_position, spacing, mass, k, rest_length,
                 damping=0.01, orientation='horizontal'):
    bodies = []
    springs = []

    # Create all bodies
    for i in range(num_bodies):
        if orientation == 'horizontal':
            pos = [start_position[0] + i * spacing, start_position[1]]
        elif orientation == 'vertical':
            pos = [start_position[0], start_position[1] + i * spacing]
        else:
            raise ValueError(f"Unknown orientation: {orientation}. Use 'horizontal' or 'vertical'")

        body = Body(mass=mass, position=pos, velocity=[0, 0])
        bodies.append(body)
    # Connect consecutive bodies with springs
    for i in range(num_bodies - 1):
        spring = Spring(bodies[i], bodies[i + 1], k, rest_length, damping)
        springs.append(spring)

    return bodies, springs