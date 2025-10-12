from engine.body import Body
from engine.world import World


# simple test using basic quantities
if __name__ == "__main__":
    ball = Body(mass=1.0, position=[0, 0], velocity=[0, 0])
    world = World(dt=0.1)
    world.add_body(ball)

    ball.apply_force([0, -9.8])

    for _ in range(10):
        world.step()
        print(f"pos={ball.position}, vel={ball.velocity}")
        ball.apply_force([0, -9.8])