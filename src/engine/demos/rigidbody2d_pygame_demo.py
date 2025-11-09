import sys
import math
import numpy as np
import pygame

from engine.rigidbody2d import PolygonShape, RigidBody2D

try:
    from engine.ui_controls import Button, Slider, InfoPanel
    HAS_UI = True
except Exception:
    HAS_UI = False


def make_box(w: float, h: float) -> np.ndarray:
    """make a box"""
    hw, hh = 0.5 * w, 0.5 * h
    return np.array([[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]], dtype=float)


def world_to_screen(p, origin, ppm):
    """world coordinates -> screen coordinates"""
    return (int(origin[0] + p[0] * ppm), int(origin[1] - p[1] * ppm))


def draw_polygon(screen, world_vertices, origin, ppm, color=(0, 130, 255)):
    pts = [world_to_screen(v, origin, ppm) for v in world_vertices]
    pygame.draw.polygon(screen, color, pts, width=0)
    pygame.draw.polygon(screen, (255, 255, 255), pts, width=2)


def main():
    pygame.init()

    SCREEN_W, SCREEN_H = 900, 640
    screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
    pygame.display.set_caption("RigidBody2D Demo")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 28)

    BACKGROUND = (22, 24, 28)
    ORIGIN = (SCREEN_W // 2, SCREEN_H // 2 + 120)  
    PPM = 100  # pixels-per-meter

    dt = 1.0 / 120.0        
    substeps = 2             
    gravity = 9.8            # m/s^2
    gravity_on = True

    shape = PolygonShape(make_box(2.0, 1.0))
    rb = RigidBody2D(
        shape=shape,
        mass=2.0,                 
        position=(0.0, 2.2),
        angle=0.0,
        velocity=(0.0, 0.0),
        angular_velocity=0.0,
    )

    ground_y = 0.0

    if HAS_UI:
        pause_btn = Button(12, 12, 110, 40, "Pause", (85, 150, 110))
        reset_btn = Button(132, 12, 110, 40, "Reset", (150, 95, 95))
        step_btn  = Button(252, 12, 110, 40, "Step",  (95, 95, 150))
        grav_btn  = Button(372, 12, 140, 40, "Gravity: ON", (120, 120, 90))
        sub_slider = Slider(12, 70, 240, 1, 8, 2, "Substeps")  # 1~8 子步
        info = InfoPanel(SCREEN_W - 260, 12)

    paused = False
    elapsed = 0.0

    def reset_state():
        rb.position[:] = (0.0, 2.2)
        rb.velocity[:] = (0.0, 0.0)
        rb.angle = 0.0
        rb.angular_velocity = 0.0
        rb.clear_forces()

    running = True
    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False

            if e.type == pygame.MOUSEBUTTONDOWN and e.button == 1:
                mx, my = e.pos
                wp = np.array([(mx - ORIGIN[0]) / PPM, -(my - ORIGIN[1]) / PPM], dtype=float)
                J = np.array([0.8, 0.4])  # N·s
                rb.apply_impulse(J, world_point=wp)

            if HAS_UI:
                if pause_btn.handle_event(e):
                    paused = not paused
                    pause_btn.text = "Play" if paused else "Pause"
                if reset_btn.handle_event(e):
                    reset_state()
                    paused = False
                    pause_btn.text = "Pause"
                    elapsed = 0.0
                if step_btn.handle_event(e) and paused:
                    if gravity_on:
                        rb.apply_force([0.0, -gravity * rb.mass])
                    rb.integrate(dt)
                    elapsed += dt
                if grav_btn.handle_event(e):
                    gravity_on = not gravity_on
                    grav_btn.text = f"Gravity: {'ON' if gravity_on else 'OFF'}"
                if e.type in (pygame.MOUSEMOTION, pygame.MOUSEBUTTONDOWN, pygame.MOUSEBUTTONUP):
                    sub_slider.handle_event(e)

            if e.type == pygame.KEYDOWN:
                if e.key == pygame.K_SPACE:
                    paused = not paused
                    if HAS_UI:
                        pause_btn.text = "Play" if paused else "Pause"
                elif e.key == pygame.K_r:
                    reset_state()
                    paused = False
                    if HAS_UI:
                        pause_btn.text = "Pause"
                    elapsed = 0.0
                elif e.key == pygame.K_g:
                    gravity_on = not gravity_on
                    if HAS_UI:
                        grav_btn.text = f"Gravity: {'ON' if gravity_on else 'OFF'}"

        if not paused:
            substeps = int(round(max(1, sub_slider.get_value()))) if HAS_UI else substeps
            for _ in range(substeps):
                if gravity_on:
                    rb.apply_force([0.0, -gravity * rb.mass]) 
                rb.integrate(dt)
                elapsed += dt

        com_y = rb.position[1]
        if com_y < ground_y + 0.52:  
            restitution = 0.25
            rb.position[1] = ground_y + 0.52
            if rb.velocity[1] < 0:
                rb.velocity[1] = -rb.velocity[1] * restitution
            rb.angular_velocity *= 0.9

        screen.fill(BACKGROUND)

        y0 = world_to_screen((0, ground_y), ORIGIN, PPM)[1]
        pygame.draw.line(screen, (100, 100, 100), (0, y0), (SCREEN_W, y0), 2)

        draw_polygon(screen, rb.world_vertices(), ORIGIN, PPM, color=(0, 140, 255))

        info_lines = [
            f"t={elapsed:.3f}s",
            f"pos=({rb.position[0]:.3f},{rb.position[1]:.3f}) m",
            f"vel=({rb.velocity[0]:.3f},{rb.velocity[1]:.3f}) m/s",
            f"theta={rb.angle:.3f} rad, omega={rb.angular_velocity:.3f} rad/s",
            f"gravity={'ON' if gravity_on else 'OFF'}  g={gravity:.1f} m/s^2",
            "LMB: impulse at mouse point",
            "SPACE: pause/play, R: reset, G: toggle gravity",
        ]

        if HAS_UI:
            info.draw(screen, elapsed, rb.position, rb.velocity, paused)
            y = 12 + 25 * 4
            txt = font.render(f"θ={rb.angle:.3f} rad, ω={rb.angular_velocity:.3f} rad/s", True, (200, 200, 200))
            screen.blit(txt, (SCREEN_W - 260, y))

            sub_slider.draw(screen)
            pause_btn.draw(screen)
            reset_btn.draw(screen)
            step_btn.draw(screen)
            grav_btn.draw(screen)
        else:
            y = 10
            for line in info_lines:
                screen.blit(font.render(line, True, (210, 210, 210)), (10, y))
                y += 22

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    sys.exit(0)


if __name__ == "__main__":
    main()
