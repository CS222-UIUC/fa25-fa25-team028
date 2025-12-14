import pygame
import sys
import os
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from engine.body import Body
from engine.spring import Spring
from engine.world import World
from engine.ui_controls import Button, Slider,InfoPanel

def main():
    pygame.init()

    SCREEN_WIDTH = 800
    SCREEN_HEIGHT = 600
    BACKGROUND_COLOR = (170, 170, 170)

    PIXELS_PER_METER = 100
    ORIGIN_X = SCREEN_WIDTH // 2
    ORIGIN_Y = SCREEN_HEIGHT // 2

    font = pygame.font.Font(None, 36)

    def physics_to_display(physics_position):
        x = ORIGIN_X + physics_position[0] * PIXELS_PER_METER
        y = ORIGIN_Y - physics_position[1] * PIXELS_PER_METER
        return (int(x), int(y))

    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Spring Physics Demo")

    clock = pygame.time.Clock()
    FPS = 60


    world = World(dt=0.016)
    fixed_body = Body(mass=10**10, position=[0, 3], velocity=[0, 0])
    moving_body = Body(mass=1.0, position=[0, 1], velocity=[0, 0])
    world.add_body(fixed_body)
    world.add_body(moving_body)
    spring = Spring(fixed_body, moving_body, k=50.0, rest_length=1.0, damping=0)
    world.add_spring(spring)

    paused = False
    play_pause_button = Button(10, SCREEN_HEIGHT - 60, 120, 50, "Pause", (100, 150, 100))
    reset_button = Button(140, SCREEN_HEIGHT - 60, 120, 50, "Reset", (150, 100, 100))
    step_button = Button(270, SCREEN_HEIGHT - 60, 120, 50, "Step", (100, 100, 150))
    speed_slider = Slider(10, 50, 150, 0.1, 3.0, 1.0, "Speed")
    info_panel = InfoPanel(SCREEN_WIDTH - 250, 10)
    elapsed_time = 0.0

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if play_pause_button.handle_event(event):
                paused = not paused
                # Update button text
                play_pause_button.text = "Play" if paused else "Pause"
                print("Paused" if paused else "Unpaused")

                # Check reset button
            if reset_button.handle_event(event):
                # Reset simulation
                fixed_body.position = np.array([0, 3], dtype=float)
                fixed_body.velocity = np.array([0, 0], dtype=float)
                moving_body.position = np.array([0, 1], dtype=float)
                moving_body.velocity = np.array([0, 0], dtype=float)
                fixed_body.clear_force()
                moving_body.clear_force()
                elapsed_time = 0.0
                paused = False
                play_pause_button.text = "Pause"
                print("Reset")
            speed_slider.handle_event(event)

            if step_button.handle_event(event):
                if paused:
                    moving_body.apply_force([0, -9.8 * moving_body.mass])
                    world.step()
                    elapsed_time += world.dt
                    print(f"Stepped to t={elapsed_time:.3f}s")
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    paused = not paused
                    play_pause_button.text = "Play" if paused else "Pause"

                if event.key == pygame.K_r:
                    fixed_body.position = np.array([0, 3], dtype=float)
                    fixed_body.velocity = np.array([0, 0], dtype=float)
                    moving_body.position = np.array([0, 1], dtype=float)
                    moving_body.velocity = np.array([0, 0], dtype=float)
                    fixed_body.clear_force()
                    moving_body.clear_force()
                    elapsed_time = 0.0
                    print("Reset")

        if not paused:
            moving_body.apply_force([0, -9.8 * moving_body.mass])
            for i in range(int(speed_slider.get_value())):
                world.step()
                elapsed_time += world.dt
        screen.fill(BACKGROUND_COLOR)
        fixed_body_pos = physics_to_display(fixed_body.position)
        moving_body_pos = physics_to_display(moving_body.position)
        pygame.draw.line(screen, (50, 50, 50), fixed_body_pos, moving_body_pos, 2)
        pygame.draw.circle(screen, (100, 100, 100), fixed_body_pos, 10)
        pygame.draw.circle(screen, (0, 100, 255), moving_body_pos, 15)

        info_panel.draw(screen, elapsed_time, moving_body.position, moving_body.velocity, paused)
        speed_slider.draw(screen)
        play_pause_button.draw(screen)
        reset_button.draw(screen)
        step_button.draw(screen)

        if paused:
            pause_text = font.render("PAUSED", True, (255, 0, 0))
            screen.blit(pause_text, (10, 10))
        pygame.display.flip()
        clock.tick(FPS)


    pygame.quit()
    sys.exit()
if __name__ == "__main__":
    main()