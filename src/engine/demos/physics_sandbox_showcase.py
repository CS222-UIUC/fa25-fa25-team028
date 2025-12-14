import sys
import os
import numpy as np
import pygame

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from engine.body import Body
from engine.spring import Spring
from engine.world import World
from engine.collisions import CollisionSystem
from engine.utils import create_chain
from engine.ui_controls import Button, Slider, InfoPanel


SCREEN_W, SCREEN_H = 900, 640
BACKGROUND = (22, 24, 28)

PPM = 80.0  # pixels per meter
ORIGIN = (SCREEN_W // 2, SCREEN_H // 2 + 80)  

GRAVITY = 9.8
AIR_DAMP = 0.985      # air
BLOCK_RADIUS = 0.25   # 


def world_to_screen(p):
    """world (x,y) -> pygame (sx, sy)"""
    return (int(ORIGIN[0] + p[0] * PPM),
            int(ORIGIN[1] - p[1] * PPM))


def draw_block(screen, body, color=(0, 140, 255)):
    x, y = body.position
    r = body.radius
    half = r * PPM
    cx, cy = world_to_screen((x, y))
    rect = pygame.Rect(0, 0, int(2 * half), int(2 * half))
    rect.center = (cx, cy)
    pygame.draw.rect(screen, color, rect)
    pygame.draw.rect(screen, (255, 255, 255), rect, 2)


def draw_spring(screen, spring, color=(220, 220, 120)):
    p1 = world_to_screen(spring.body1.position)
    p2 = world_to_screen(spring.body2.position)
    pygame.draw.line(screen, color, p1, p2, 2)


def pick_body(bodies, world_point, max_dist=0.5):
    best = None
    best_d2 = max_dist ** 2
    wp = np.asarray(world_point, dtype=float)
    for b in bodies:
        d2 = float(np.sum((b.position - wp) ** 2))
        if d2 < best_d2:
            best_d2 = d2
            best = b
    return best


def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
    pygame.display.set_caption("Physics Sandbox Showcase")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 24)

    collision_sys = CollisionSystem(
        floor_y=-2.5,
        floor_enabled=True,
        wall_left=-4.0,
        wall_right=4.0,
        walls_enabled=True,
        bounce_damping=0.3,
    )

    BASE_DT = 1.0 / 120.0  
    world = World(dt=BASE_DT, collision_system=collision_sys)
    b1 = Body(mass=1.0, position=[-1.0, 1.5], velocity=[0.0, 0.0],
              radius=BLOCK_RADIUS, restitution=0.0)
    b2 = Body(mass=1.0, position=[1.0, 1.5], velocity=[0.0, 0.0],
              radius=BLOCK_RADIUS, restitution=0.0)
    world.add_body(b1)
    world.add_body(b2)

    init_spring = Spring(b1, b2, k=15.0,
                         rest_length=np.linalg.norm(b2.position - b1.position),
                         damping=0.5)
    world.add_spring(init_spring)

    pause_btn = Button(12, 12, 90, 32, "Pause", (85, 150, 110))
    reset_btn = Button(112, 12, 90, 32, "Reset", (150, 95, 95))
    drop_btn  = Button(212, 12, 120, 32, "Drop block", (95, 140, 200))
    spring_btn = Button(342, 12, 140, 32, "Link Spring", (140, 120, 90))
    chain_btn  = Button(492, 12, 150, 32, "Add Chain", (120, 95, 150))
    grav_btn   = Button(652, 12, 140, 32, "Gravity: ON", (120, 120, 90))

    sub_slider = Slider(12, 60, 240, 1, 8, 2, "Speed")
    info_panel = InfoPanel(SCREEN_W - 250, 12)

    paused = False
    gravity_on = True
    elapsed = 0.0

    grabbed_body = None     
    mouse_world = np.zeros(2)

    spring_mode = False
    spring_first_body = None

    def reset_world():
        world.bodies.clear()
        world.springs.clear()

        bb1 = Body(mass=1.0, position=[-1.0, 1.5], velocity=[0.0, 0.0],
                   radius=BLOCK_RADIUS, restitution=0.0)
        bb2 = Body(mass=1.0, position=[1.0, 1.5], velocity=[0.0, 0.0],
                   radius=BLOCK_RADIUS, restitution=0.0)
        world.add_body(bb1)
        world.add_body(bb2)
        sp = Spring(bb1, bb2, k=15.0,
                    rest_length=np.linalg.norm(bb2.position - bb1.position),
                    damping=0.5)
        world.add_spring(sp)

        return bb1  

    main_body = b1  

    running = True
    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False

            if e.type in (pygame.MOUSEMOTION, pygame.MOUSEBUTTONDOWN, pygame.MOUSEBUTTONUP):
                mx, my = e.pos
                mouse_world = np.array(
                    [(mx - ORIGIN[0]) / PPM,
                     -(my - ORIGIN[1]) / PPM],
                    dtype=float
                )
                sub_slider.handle_event(e)

            if e.type == pygame.MOUSEBUTTONDOWN and e.button == 1:
                if not any(btn.rect.collidepoint(e.pos) for btn in
                           [pause_btn, reset_btn, drop_btn, spring_btn, chain_btn, grav_btn]):
                    if spring_mode:
                        target = pick_body(world.bodies, mouse_world, max_dist=0.7)
                        if target is not None:
                            if spring_first_body is None:
                                spring_first_body = target
                            else:
                                rest_len = np.linalg.norm(
                                    target.position - spring_first_body.position
                                )
                                new_spring = Spring(
                                    spring_first_body,
                                    target,
                                    k=20.0,
                                    rest_length=rest_len,
                                    damping=0.7
                                )
                                world.add_spring(new_spring)
                                spring_first_body = None
                                spring_mode = False
                    else:
                        grabbed_body = pick_body(world.bodies, mouse_world, max_dist=0.6)

            if e.type == pygame.MOUSEBUTTONUP and e.button == 1:
                grabbed_body = None

            if e.type == pygame.KEYDOWN:
                if e.key == pygame.K_SPACE:
                    paused = not paused
                    pause_btn.text = "Play" if paused else "Pause"
                elif e.key == pygame.K_r:
                    main_body = reset_world()
                    paused = False
                    pause_btn.text = "Pause"
                    elapsed = 0.0
                elif e.key == pygame.K_g:
                    gravity_on = not gravity_on
                    grav_btn.text = f"Gravity: {'ON' if gravity_on else 'OFF'}"

            if pause_btn.handle_event(e):
                paused = not paused
                pause_btn.text = "Play" if paused else "Pause"

            if reset_btn.handle_event(e):
                main_body = reset_world()
                paused = False
                pause_btn.text = "Pause"
                elapsed = 0.0

            if drop_btn.handle_event(e):
                new_body = Body(
                    mass=1.0,
                    position=[0.0, 3.0],
                    velocity=[0.0, 0.0],
                    radius=BLOCK_RADIUS,
                    restitution=0.0
                )
                world.add_body(new_body)

            if spring_btn.handle_event(e):
                spring_mode = True
                spring_first_body = None

            if chain_btn.handle_event(e):
                chain_bodies, chain_springs = create_chain(
                    num_bodies=6,
                    start_position=[-2.5, 2.8],
                    spacing=0.5,
                    mass=0.8,
                    k=25.0,
                    rest_length=0.5,
                    damping=0.7,
                    orientation='horizontal'
                )
                for b in chain_bodies:
                    b.radius = BLOCK_RADIUS * 0.9
                    b.restitution = 0.0
                    world.add_body(b)
                for s in chain_springs:
                    world.add_spring(s)

            if grav_btn.handle_event(e):
                gravity_on = not gravity_on
                grav_btn.text = f"Gravity: {'ON' if gravity_on else 'OFF'}"

        if not paused:
            speed_raw = sub_slider.get_value()
            speed_factor = speed_raw / 2.0     # 2 -> 1x, 4 -> 2x, 8 -> 4x

            world.dt = BASE_DT * speed_factor

            for body in world.bodies:
                if body.mass >= 1e9:
                    continue
                if body is grabbed_body:
                    body.position = mouse_world.copy()
                    body.velocity[:] = 0.0
                    body.clear_force()
                else:
                    if gravity_on:
                        body.apply_force([0.0, -GRAVITY * body.mass])

            world.step()

            for body in world.bodies:
                body.velocity *= AIR_DAMP

            elapsed += world.dt


        screen.fill(BACKGROUND)

        floor_y = collision_sys.floor_y
        y0 = world_to_screen((0.0, floor_y))[1]
        pygame.draw.line(screen, (100, 100, 100), (0, y0), (SCREEN_W, y0), 2)

        if collision_sys.walls_enabled:
            if collision_sys.wall_left is not None:
                x_l = world_to_screen((collision_sys.wall_left, 0.0))[0]
                pygame.draw.line(screen, (100, 100, 100),
                                 (x_l, 0), (x_l, SCREEN_H), 2)
            if collision_sys.wall_right is not None:
                x_r = world_to_screen((collision_sys.wall_right, 0.0))[0]
                pygame.draw.line(screen, (100, 100, 100),
                                 (x_r, 0), (x_r, SCREEN_H), 2)

        # springs
        for s in world.springs:
            draw_spring(screen, s)

        # blocks
        colors = [(0, 140, 255), (255, 140, 0), (100, 220, 120)]
        for i, b in enumerate(world.bodies):
            draw_block(screen, b, color=colors[i % len(colors)])

        if spring_mode:
            msg = "Spring mode: click two blocks to connect"
            tip = font.render(msg, True, (230, 230, 160))
            screen.blit(tip, (12, SCREEN_H - 30))

        info_panel.draw(screen, elapsed, main_body.position, main_body.velocity, paused)

        # UI 
        sub_slider.draw(screen)
        pause_btn.draw(screen)
        reset_btn.draw(screen)
        drop_btn.draw(screen)
        spring_btn.draw(screen)
        chain_btn.draw(screen)
        grav_btn.draw(screen)

        pygame.display.flip()
        clock.tick(60)


if __name__ == "__main__":
    main()
