import pygame
import sys
import os
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from engine.body import Body
from engine.world import World
from engine.collisions import CollisionSystem
from engine.ui_controls import Button, Slider, InfoPanel

def main():
    pygame.init()

    SCREEN_WIDTH = 800
    SCREEN_HEIGHT = 600
    BACKGROUND_COLOR = (255, 255, 255)

    PIXELS_PER_METER = 100
    ORIGIN_X = SCREEN_WIDTH // 2
    ORIGIN_Y = SCREEN_HEIGHT // 2

    font = pygame.font.Font(None, 36)

    def physics_to_display(physics_position):
        x = ORIGIN_X + physics_position[0] * PIXELS_PER_METER
        y = ORIGIN_Y - physics_position[1] * PIXELS_PER_METER
        return (int(x), int(y))

    def display_to_physics(screen_pos):
        x = (screen_pos[0] - ORIGIN_X) / PIXELS_PER_METER
        y = (ORIGIN_Y - screen_pos[1]) / PIXELS_PER_METER
        return np.array([x, y], dtype=float)

    def get_body_at_position(pos):
        physics_pos = display_to_physics(pos)
        for body in world.bodies:
            if body.is_static:
                continue
            distance = np.linalg.norm(body.position - physics_pos)
            if distance < body.radius:
                return body
        return None

    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Drag & Collision Demo")

    clock = pygame.time.Clock()
    FPS = 60

    collision_system = CollisionSystem(penetration_slop=1e-3)
    world = World(dt=0.016, collision_system=collision_system)

    BLOCK_RADIUS = 0.3
    BLOCK_MASS = 1.0

    def create_initial_bodies():
        world.bodies.clear()
        
        floor = Body(mass=10**10, position=[0, -2.5], velocity=[0, 0], 
                     radius=5.0, restitution=0.3, is_static=True)
        world.add_body(floor)
        
        positions = [
            [-1.5, 0.5],
            [0, 0.5],
            [1.5, 0.5],
            [-0.75, -0.5],
            [0.75, -0.5],
        ]
        
        for pos in positions:
            block = Body(mass=BLOCK_MASS, position=pos, 
                        velocity=[0, 0], radius=BLOCK_RADIUS, restitution=0.5)
            world.add_body(block)
        
        return world.bodies[1] if len(world.bodies) > 1 else None

    main_body = create_initial_bodies()
    dragged_body = None
    drag_offset = None

    paused = False
    play_pause_button = Button(10, SCREEN_HEIGHT - 60, 120, 50, "Pause", (76, 175, 80))
    reset_button = Button(140, SCREEN_HEIGHT - 60, 120, 50, "Reset", (244, 67, 54))
    step_button = Button(270, SCREEN_HEIGHT - 60, 120, 50, "Step", (33, 150, 243))
    add_block_button = Button(400, SCREEN_HEIGHT - 60, 140, 50, "Add Block", (156, 39, 176))
    clear_button = Button(550, SCREEN_HEIGHT - 60, 120, 50, "Clear", (255, 152, 0))
    speed_slider = Slider(10, 50, 150, 0.1, 3.0, 1.0, "Speed")
    info_panel = InfoPanel(SCREEN_WIDTH - 250, 10)
    elapsed_time = 0.0

    def reset_simulation():
        nonlocal main_body, dragged_body, drag_offset
        main_body = create_initial_bodies()
        dragged_body = None
        drag_offset = None
        return 0.0

    def add_block_at_mouse(mouse_pos):
        physics_pos = display_to_physics(mouse_pos)
        new_block = Body(mass=BLOCK_MASS, position=physics_pos, 
                       velocity=[0, 0], radius=BLOCK_RADIUS, restitution=0.5)
        world.add_body(new_block)
        return new_block

    def clear_all_blocks():
        world.bodies = [b for b in world.bodies if b.is_static]

    running = True
    while running:
        mouse_pos = pygame.mouse.get_pos()
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if not any(btn.rect.collidepoint(event.pos) for btn in 
                          [play_pause_button, reset_button, step_button, 
                           add_block_button, clear_button]):
                    body = get_body_at_position(event.pos)
                    if body:
                        dragged_body = body
                        physics_pos = display_to_physics(event.pos)
                        drag_offset = physics_pos - body.position
                    else:
                        add_block_at_mouse(event.pos)
            
            if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                if dragged_body:
                    dragged_body.velocity[:] = 0.0
                dragged_body = None
                drag_offset = None

            if play_pause_button.handle_event(event):
                paused = not paused
                play_pause_button.text = "Play" if paused else "Pause"

            if reset_button.handle_event(event):
                elapsed_time = reset_simulation()
                paused = False
                play_pause_button.text = "Pause"

            if add_block_button.handle_event(event):
                add_block_at_mouse(mouse_pos)

            if clear_button.handle_event(event):
                clear_all_blocks()

            if step_button.handle_event(event):
                if paused:
                    world.step()
                    elapsed_time += world.dt

            speed_slider.handle_event(event)

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    paused = not paused
                    play_pause_button.text = "Play" if paused else "Pause"
                if event.key == pygame.K_r:
                    elapsed_time = reset_simulation()
                if event.key == pygame.K_c:
                    clear_all_blocks()

        if dragged_body and drag_offset is not None:
            physics_pos = display_to_physics(mouse_pos)
            new_pos = physics_pos - drag_offset
            
            for body in world.bodies:
                if body == dragged_body or body.is_static:
                    continue
                distance = np.linalg.norm(new_pos - body.position)
                min_distance = body.radius + dragged_body.radius + 0.01
                if distance < min_distance:
                    direction = (new_pos - body.position)
                    if np.linalg.norm(direction) > 1e-8:
                        direction = direction / np.linalg.norm(direction)
                        overlap = min_distance - distance
                        new_pos += direction * overlap
            
            dragged_body.position = new_pos
            dragged_body.velocity[:] = 0.0
            dragged_body.clear_force()

        if not paused:
            speed_factor = speed_slider.get_value()
            world.dt = 0.016 * speed_factor
            
            for body in world.bodies:
                if not body.is_static and body.mass < 1e9 and body != dragged_body:
                    body.apply_force([0, -9.8 * body.mass])
            
            if dragged_body is None:
                world.step()
            else:
                from engine.integrator import euler_step
                for spring in world.springs:
                    spring.apply_forces()
                AIR_RESISTANCE = 0.98
                for body in world.bodies:
                    if body == dragged_body or body.is_static:
                        continue
                    if body.mass < 1e9:
                        euler_step(body, world.dt)
                        body.velocity *= AIR_RESISTANCE
                    else:
                        body.clear_force()
                temp_dragged = dragged_body
                world.bodies.remove(dragged_body)
                world.collision_system.step(world.bodies)
                world.bodies.append(temp_dragged)
                dragged_body = temp_dragged
            
            elapsed_time += world.dt

        screen.fill(BACKGROUND_COLOR)

        floor_y = physics_to_display((0, -2.5))[1]
        pygame.draw.line(screen, (100, 100, 100), (0, floor_y), (SCREEN_WIDTH, floor_y), 3)

        colors = [(0, 140, 255), (255, 140, 0), (100, 220, 120), 
                  (255, 100, 150), (150, 100, 255), (100, 200, 255),
                  (255, 200, 100), (200, 100, 255)]
        
        for i, body in enumerate(world.bodies):
            if body.is_static:
                continue
            pos = physics_to_display(body.position)
            radius_px = int(body.radius * PIXELS_PER_METER)
            color = colors[i % len(colors)]
            
            if body == dragged_body:
                pygame.draw.circle(screen, color, pos, radius_px + 2)
                pygame.draw.circle(screen, (255, 255, 0), pos, radius_px + 2, 3)
            else:
                pygame.draw.circle(screen, color, pos, radius_px)
            pygame.draw.circle(screen, (50, 50, 50), pos, radius_px, 2)

        if main_body:
            info_panel.draw(screen, elapsed_time, main_body.position, main_body.velocity, paused)

        block_count = len([b for b in world.bodies if not b.is_static])
        count_text = font.render(f"Blocks: {block_count}", True, (50, 50, 50))
        screen.blit(count_text, (10, 10))

        if dragged_body:
            hint_text = font.render("Dragging...", True, (255, 152, 0))
            screen.blit(hint_text, (10, 50))
        else:
            hint_text = font.render("Click to drag, Click empty to add", True, (150, 150, 150))
            screen.blit(hint_text, (10, 50))

        play_pause_button.draw(screen)
        reset_button.draw(screen)
        step_button.draw(screen)
        add_block_button.draw(screen)
        clear_button.draw(screen)
        speed_slider.draw(screen)

        if paused:
            pause_text = font.render("PAUSED", True, (255, 0, 0))
            screen.blit(pause_text, (SCREEN_WIDTH // 2 - 70, 10))

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()

