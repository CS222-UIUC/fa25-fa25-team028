import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from engine.utils import create_chain
from engine.world import World

bodies, springs = create_chain(
    num_bodies=5,
    start_position=[0, 0],
    spacing=1.0,
    mass=1.0,
    k=10.0,
    rest_length=1.0,
    damping=0.01,
    orientation='horizontal'
)

world = World(dt=0.01)

# Add all bodies and springs to world
for body in bodies:
    world.add_body(body)

for spring in springs:
    world.add_spring(spring)

# Apply gravity to all bodies
gravity = [0, -9.8]

print("Chain Demo - Horizontal 5-body chain with gravity\n")
print("Initial positions:")
for i, body in enumerate(bodies):
    print(f"Body {i}: {body.position}")

print("\nRunning simulation for 100 steps...\n")

for step in range(100):
    # Apply gravity to each body
    for body in bodies:
        body.apply_force(gravity)

    world.step()

    # Print every 20 steps
    if step % 20 == 0:
        print(f"Step {step}:")
        for i, body in enumerate(bodies):
            print(f"  Body {i}: x={body.position[0]:.2f}, y={body.position[1]:.2f}")

print("\nFinal positions:")
for i, body in enumerate(bodies):
    print(f"Body {i}: {body.position}")