import pygame
import sys
import os
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from engine.body import Body
from engine.spring import Spring
from engine.world import World
from engine.utils import create_chain
from engine.ui_controls import Button, Slider, InfoPanel
from engine.collisions import CollisionSystem


def main():
    # Initialize Pygame
    pygame.init()

    # Screen settings
    SCREEN_WIDTH = 800
    SCREEN_HEIGHT = 600
    BACKGROUND_COLOR = (170, 170, 170)
    PIXELS_PER_METER = 50
    ORIGIN_X = SCREEN_WIDTH // 2
    ORIGIN_Y = SCREEN_HEIGHT // 2

    font = pygame.font.Font(None, 36)

    def physics_to_display(physics_position):
        """Convert physics coordinates to screen coordinates"""
        x = ORIGIN_X + physics_position[0] * PIXELS_PER_METER
        y = ORIGIN_Y - physics_position[1] * PIXELS_PER_METER
        return (int(x), int(y))

    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Spring Chain Demo")
    clock = pygame.time.Clock()
    FPS = 60

    # Create physics world with collision system (floor and walls enabled)
    collision_system = CollisionSystem(
        floor_y=-4.0,
        floor_enabled=True,
        wall_left=-8.0,
        wall_right=8.0,
        walls_enabled=True,
        bounce_damping=0.7
    )
    world = World(dt=0.01, collision_system=collision_system)

    bodies, springs = create_chain(
        num_bodies=8,
        start_position=[-3.5, 2],
        spacing=1.0,
        mass=1.0,
        k=30.0,
        rest_length=1.0,
        damping=0.2,
        orientation='horizontal'
    )

    for body in bodies:
        world.add_body(body)
    for spring in springs:
        world.add_spring(spring)

    # Simulation state
    paused = False
    apply_gravity = True
    fix_endpoints = False
    elapsed_time = 0.0
    dragging_body = None

    # Store initial state for reset
    initial_positions = [body.position.copy() for body in bodies]
    initial_velocities = [body.velocity.copy() for body in bodies]

    # UI Controls
    play_pause_button = Button(10, SCREEN_HEIGHT - 60, 120, 50, "Pause", (100, 150, 100))
    reset_button = Button(140, SCREEN_HEIGHT - 60, 120, 50, "Reset", (150, 100, 100))
    step_button = Button(270, SCREEN_HEIGHT - 60, 120, 50, "Step", (100, 100, 150))
    gravity_button = Button(400, SCREEN_HEIGHT - 60, 120, 50, "Gravity: ON", (100, 100, 200))
    bridge_button = Button(530, SCREEN_HEIGHT - 60, 120, 50, "Bridge: OFF", (200, 100, 100))
    floor_button = Button(660, SCREEN_HEIGHT - 60, 120, 50, "Floor: ON", (150, 100, 150))

    speed_slider = Slider(10, 50, 150, 0.1, 3.0, 1.0, "Speed")
    info_panel = InfoPanel(SCREEN_WIDTH - 250, 10)

    def reset_simulation():
        """Reset all bodies to initial positions and velocities"""
        nonlocal elapsed_time
        for i, body in enumerate(bodies):
            body.position = initial_positions[i].copy()
            body.velocity = initial_velocities[i].copy()
            body.clear_force()
        elapsed_time = 0.0

    def toggle_bridge_mode():
        """Toggle fixed endpoints for bridge simulation"""
        nonlocal fix_endpoints
        fix_endpoints = not fix_endpoints

        if fix_endpoints:
            bodies[0].mass = 10 ** 10
            bodies[-1].mass = 10 ** 10
            bodies[0].velocity = np.zeros(2, dtype=float)
            bodies[-1].velocity = np.zeros(2, dtype=float)
        else:
            bodies[0].mass = 1.0
            bodies[-1].mass = 1.0

        bridge_button.text = f"Bridge: {'ON' if fix_endpoints else 'OFF'}"

    # Main game loop
    running = True
    while running:
        # Event handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            # Button events
            if play_pause_button.handle_event(event):
                paused = not paused
                play_pause_button.text = "Play" if paused else "Pause"

            if reset_button.handle_event(event):
                reset_simulation()
                paused = False
                play_pause_button.text = "Pause"

            if step_button.handle_event(event):
                if paused:
                    if apply_gravity:
                        for body in bodies:
                            if not (fix_endpoints and (body == bodies[0] or body == bodies[-1])):
                                body.apply_force([0, -9.8 * body.mass])
                    world.step()
                    elapsed_time += world.dt

            if gravity_button.handle_event(event):
                apply_gravity = not apply_gravity
                gravity_button.text = f"Gravity: {'ON' if apply_gravity else 'OFF'}"

            if bridge_button.handle_event(event):
                toggle_bridge_mode()

            if floor_button.handle_event(event):
                world.collision_system.floor_enabled = not world.collision_system.floor_enabled
                floor_button.text = f"Floor: {'ON' if world.collision_system.floor_enabled else 'OFF'}"

            speed_slider.handle_event(event)

            # Keyboard shortcuts
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    paused = not paused
                    play_pause_button.text = "Play" if paused else "Pause"
                elif event.key == pygame.K_r:
                    reset_simulation()
                elif event.key == pygame.K_g:
                    apply_gravity = not apply_gravity
                    gravity_button.text = f"Gravity: {'ON' if apply_gravity else 'OFF'}"
                elif event.key == pygame.K_b:
                    toggle_bridge_mode()
                elif event.key == pygame.K_f:
                    world.collision_system.floor_enabled = not world.collision_system.floor_enabled
                    floor_button.text = f"Floor: {'ON' if world.collision_system.floor_enabled else 'OFF'}"

            # Mouse dragging
            if event.type == pygame.MOUSEBUTTONDOWN:
                mouse_pos = pygame.mouse.get_pos()
                for body in bodies:
                    body_screen_pos = physics_to_display(body.position)
                    dist_squared = (mouse_pos[0] - body_screen_pos[0]) ** 2 + (mouse_pos[1] - body_screen_pos[1]) ** 2
                    if dist_squared < 225:  # 15^2 = 225 (click radius)
                        dragging_body = body
                        paused = True
                        play_pause_button.text = "Play"
                        break

            elif event.type == pygame.MOUSEBUTTONUP:
                dragging_body = None

            elif event.type == pygame.MOUSEMOTION and dragging_body is not None:
                mouse_pos = pygame.mouse.get_pos()
                physics_x = (mouse_pos[0] - ORIGIN_X) / PIXELS_PER_METER
                physics_y = (ORIGIN_Y - mouse_pos[1]) / PIXELS_PER_METER
                dragging_body.position = np.array([physics_x, physics_y], dtype=float)
                dragging_body.velocity = np.zeros(2, dtype=float)

        # Physics update
        if not paused and dragging_body is None:
            # Apply gravity
            if apply_gravity:
                for body in bodies:
                    if not (fix_endpoints and (body == bodies[0] or body == bodies[-1])):
                        body.apply_force([0, -9.8 * body.mass])

            # Run simulation steps
            for _ in range(int(speed_slider.get_value())):
                world.step()
                elapsed_time += world.dt

        # Rendering
        screen.fill(BACKGROUND_COLOR)

        # Draw floor
        if world.collision_system.floor_enabled:
            floor_screen_y = physics_to_display([0, world.collision_system.floor_y])[1]
            pygame.draw.line(screen, (100, 50, 50), (0, floor_screen_y),
                             (SCREEN_WIDTH, floor_screen_y), 4)

        # Draw walls
        if world.collision_system.walls_enabled:
            left_wall_x = physics_to_display([world.collision_system.wall_left, 0])[0]
            right_wall_x = physics_to_display([world.collision_system.wall_right, 0])[0]
            pygame.draw.line(screen, (50, 100, 50), (left_wall_x, 0),
                             (left_wall_x, SCREEN_HEIGHT), 4)
            pygame.draw.line(screen, (50, 100, 50), (right_wall_x, 0),
                             (right_wall_x, SCREEN_HEIGHT), 4)

        # Draw springs
        for spring in springs:
            pos1 = physics_to_display(spring.body1.position)
            pos2 = physics_to_display(spring.body2.position)
            pygame.draw.line(screen, (50, 50, 50), pos1, pos2, 3)

        # Draw bodies
        for i, body in enumerate(bodies):
            body_pos = physics_to_display(body.position)
            is_fixed = fix_endpoints and (i == 0 or i == len(bodies) - 1)
            color = (200, 50, 50) if is_fixed else (0, 100, 255)
            pygame.draw.circle(screen, color, body_pos, 12)

        # Draw UI elements
        info_panel.draw(screen, elapsed_time, bodies[len(bodies) // 2].position,
                        bodies[len(bodies) // 2].velocity, paused)
        speed_slider.draw(screen)
        play_pause_button.draw(screen)
        reset_button.draw(screen)
        step_button.draw(screen)
        gravity_button.draw(screen)
        bridge_button.draw(screen)
        floor_button.draw(screen)

        # Draw instructions
        instructions_font = pygame.font.Font(None, 20)
        instructions = [
            "Click and drag bodies to move them",
            "SPACE: Play/Pause | R: Reset | G: Gravity | B: Bridge | F: Floor"
        ]
        for i, instruction in enumerate(instructions):
            text = instructions_font.render(instruction, True, (50, 50, 50))
            screen.blit(text, (10, 10 + i * 20))

        # Draw paused indicator
        if paused and dragging_body is None:
            pause_text = font.render("PAUSED", True, (255, 0, 0))
            screen.blit(pause_text, (SCREEN_WIDTH // 2 - 80, 10))

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()