import pygame
import sys
import os
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from engine.body import Body
from engine.world import World
from engine.ui_controls import Button, InfoPanel

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

    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Collision Physics Demo")

    clock = pygame.time.Clock()
    FPS = 60

    world = World(dt=0.016)
    
    body1 = Body(mass=1.0, position=[-2, 0], velocity=[2, 0], radius=0.3, restitution=0.9)
    body2 = Body(mass=1.0, position=[2, 0], velocity=[-2, 0], radius=0.3, restitution=0.9)
    
    wall = Body(mass=10**10, position=[0, -2], velocity=[0, 0], radius=0.5, restitution=0.8, is_static=True)
    
    world.add_body(body1)
    world.add_body(body2)
    world.add_body(wall)

    paused = False
    play_pause_button = Button(10, SCREEN_HEIGHT - 60, 120, 50, "Pause", (76, 175, 80))
    reset_button = Button(140, SCREEN_HEIGHT - 60, 120, 50, "Reset", (244, 67, 54))
    step_button = Button(270, SCREEN_HEIGHT - 60, 120, 50, "Step", (33, 150, 243))
    info_panel = InfoPanel(SCREEN_WIDTH - 250, 10)
    elapsed_time = 0.0

    def reset_simulation():
        body1.position = np.array([-2, 0], dtype=float)
        body1.velocity = np.array([2, 0], dtype=float)
        body2.position = np.array([2, 0], dtype=float)
        body2.velocity = np.array([-2, 0], dtype=float)
        wall.position = np.array([0, -2], dtype=float)
        wall.velocity = np.array([0, 0], dtype=float)
        body1.clear_force()
        body2.clear_force()
        wall.clear_force()
        return 0.0

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
            if play_pause_button.handle_event(event):
                paused = not paused
                play_pause_button.text = "Play" if paused else "Pause"

            if reset_button.handle_event(event):
                elapsed_time = reset_simulation()
                paused = False
                play_pause_button.text = "Pause"

            if step_button.handle_event(event):
                if paused:
                    world.step()
                    elapsed_time += world.dt

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    paused = not paused
                    play_pause_button.text = "Play" if paused else "Pause"
                if event.key == pygame.K_r:
                    elapsed_time = reset_simulation()

        if not paused:
            world.step()
            elapsed_time += world.dt

        screen.fill(BACKGROUND_COLOR)

        body1_pos = physics_to_display(body1.position)
        body2_pos = physics_to_display(body2.position)
        wall_pos = physics_to_display(wall.position)

        pygame.draw.circle(screen, (0, 100, 255), body1_pos, int(body1.radius * PIXELS_PER_METER))
        pygame.draw.circle(screen, (255, 100, 0), body2_pos, int(body2.radius * PIXELS_PER_METER))
        
        pygame.draw.circle(screen, (100, 100, 100), wall_pos, int(wall.radius * PIXELS_PER_METER))

        info_panel.draw(screen, elapsed_time, body1.position, body1.velocity, paused)
        
        momentum1 = body1.momentum()
        momentum2 = body2.momentum()
        total_momentum = momentum1 + momentum2
        momentum_text = font.render(f"Total p: ({total_momentum[0]:.2f}, {total_momentum[1]:.2f})", True, (50, 50, 50))
        screen.blit(momentum_text, (10, 10))

        play_pause_button.draw(screen)
        reset_button.draw(screen)
        step_button.draw(screen)

        if paused:
            pause_text = font.render("PAUSED", True, (255, 0, 0))
            screen.blit(pause_text, (SCREEN_WIDTH // 2 - 70, 10))

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()

