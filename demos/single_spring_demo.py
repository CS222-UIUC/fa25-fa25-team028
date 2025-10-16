import os,sys

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SRC_DIR = os.path.join(PROJECT_ROOT, "src")
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

# not Optional, setting path to src
from engine.body import Body
from engine.spring import Spring
from engine.world import World
import numpy as np
import math

def approximated_period(mass,k):
    return 2 * math.pi * math.sqrt(mass / k)
def actual_period(positions_input,dt):
    peaks = []
    for w in range(1,len(positions_input)-1):
        if (positions_input[w - 1] < positions_input[w]) and (positions_input[w] > positions_input[w + 1]):
            peaks.append(w)
    if len(peaks) >= 2:
        time_between_peaks = (peaks[-1]-peaks[0])/(len(peaks)-1)
        return (time_between_peaks * dt) / 2
    return None
fixed_body = Body(mass=10 ** 10, position=[0, 0], velocity=[0, 0])
moving_body = Body(mass=1, position=[2,0],velocity=[0,0])

spring = Spring(fixed_body, moving_body,10,1,0)
world = World(0.01)
world.add_body(fixed_body)
world.add_body(moving_body)
world.add_spring(spring)
positions = []
for steps_index in range(5000):
    world.step()
    positions.append(moving_body.position[0])
for test1_index in range(50):
    print(f"moving position {test1_index}:   {positions[test1_index]:.2f}")
for test2_index in range(0,500,50):
    print(f"moving position {test2_index}:   {positions[test2_index]:.2f}")
print(f"max position:   {np.max(positions):.2}")
print(f"min position:   {np.min(positions):.2}")
approximated_T_val = approximated_period(moving_body.mass, spring.k)
actual_T_val = actual_period(positions, world.dt)
if actual_period:
    print(f"measured period:   {actual_T_val:.2f} seconds")
    error = (np.abs(approximated_T_val - actual_T_val) / approximated_T_val) * 100
    print(f"Error: {error:.1f}%")

