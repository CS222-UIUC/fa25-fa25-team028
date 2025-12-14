import sys
import os
import math
import numpy as np
import pygame

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from engine.rigidbody2d import PolygonShape, RigidBody2D, cross2D

REST_COEF = 0.2

try:
    from engine.ui_controls import Button, Slider, InfoPanel
    HAS_UI = True
except Exception:
    HAS_UI = False


def make_box(w: float, h: float) -> np.ndarray:
    """make a box (local vertices, centered at origin)"""
    hw, hh = 0.5 * w, 0.5 * h
    return np.array([[-hw, -hh],
                     [ hw, -hh],
                     [ hw,  hh],
                     [-hw,  hh]], dtype=float)


def world_to_screen(p, origin, ppm):
    """world coordinates -> screen coordinates"""
    return (int(origin[0] + p[0] * ppm),
            int(origin[1] - p[1] * ppm))


def draw_polygon(screen, world_vertices, origin, ppm, color=(0, 130, 255)):
    pts = [world_to_screen(v, origin, ppm) for v in world_vertices]
    pygame.draw.polygon(screen, color, pts, width=0)
    pygame.draw.polygon(screen, (255, 255, 255), pts, width=2)


# helpers for collision

def collide_with_ground(rb: RigidBody2D, ground_y: float, restitution: float = REST_COEF):
    """Rigid body vs infinite horizontal ground at y = ground_y."""
    wv = rb.world_vertices()

    below = wv[:, 1] < ground_y
    if not np.any(below):
        return

    contacts = wv[below]

    penetrations = ground_y - contacts[:, 1]
    max_pen = np.max(penetrations)
    rb.position[1] += max_pen
    rb._update_rotation()

    n = np.array([0.0, 1.0])  # ground normal: up

    mu = 0.5 # friction added

    for p in contacts:
        r = p - rb.position  # from COM to contact

        # contact point velocity: v + ω × r
        v_contact = rb.velocity + np.array(
            [-rb.angular_velocity * r[1],
              rb.angular_velocity * r[0]]
        )
        v_rel_n = np.dot(v_contact, n)
        if v_rel_n >= 0.0:
            continue  # not moving into the ground

        rn = cross2D(r, n)
        inv_mass_eff = rb.inv_mass + (rn * rn) * rb.inv_inertia

        jn = -(1.0 + restitution) * v_rel_n / inv_mass_eff
        impulse = jn * n

        rb.apply_impulse(impulse, world_point=p)
        vt = v_contact - v_rel_n * n
        vt_len = np.linalg.norm(vt)
        if vt_len < 1e-6:
            continue  # no sliding

        t = vt / vt_len  

        rt = cross2D(r, t)
        inv_mass_eff_t = rb.inv_mass + (rt * rt) * rb.inv_inertia

        v_rel_t = np.dot(v_contact, t)
        jt = -v_rel_t / (inv_mass_eff_t + 1e-12)

        max_jt = mu * jn
        jt = np.clip(jt, -max_jt, max_jt)

        impulse_t = jt * t
        rb.apply_impulse(impulse_t, world_point=p)


def approx_radius(rb: RigidBody2D) -> float:
    """Approximate bounding circle radius for a body."""
    wv = rb.world_vertices()
    rel = wv - rb.position
    return float(np.linalg.norm(rel, axis=1).max())


def collide_bodies(a: RigidBody2D, b: RigidBody2D, restitution: float = REST_COEF):
    """
    simplified rigid body collilsion
    12.4 update: friction added to avoid weird collision
    """
    wv_a = a.world_vertices()
    wv_b = b.world_vertices()

    a_min = wv_a.min(axis=0)
    a_max = wv_a.max(axis=0)
    b_min = wv_b.min(axis=0)
    b_max = wv_b.max(axis=0)

    overlap_x = min(a_max[0], b_max[0]) - max(a_min[0], b_min[0])
    overlap_y = min(a_max[1], b_max[1]) - max(a_min[1], b_min[1])

    if overlap_x <= 0.0 or overlap_y <= 0.0:
        return

    delta = b.position - a.position

    if overlap_x < overlap_y:
        if delta[0] >= 0.0:
            n = np.array([1.0, 0.0])
        else:
            n = np.array([-1.0, 0.0])
        penetration = overlap_x
    else:
        if delta[1] >= 0.0:
            n = np.array([0.0, 1.0])
        else:
            n = np.array([0.0, -1.0])
        penetration = overlap_y

    correction = 0.8 * penetration / (a.inv_mass + b.inv_mass + 1e-12) * n
    a.position -= a.inv_mass * correction
    b.position += b.inv_mass * correction

    pa = a.position
    pb = b.position
    contact = 0.5 * (pa + pb)

    ra_vec = contact - pa
    rb_vec = contact - pb

    vA = a.velocity + np.array(
        [-a.angular_velocity * ra_vec[1],
          a.angular_velocity * ra_vec[0]]
    )
    vB = b.velocity + np.array(
        [-b.angular_velocity * rb_vec[1],
          b.angular_velocity * rb_vec[0]]
    )
    v_rel = vB - vA
    v_rel_n = np.dot(v_rel, n)
    if v_rel_n >= 0.0:
        return

    ra_n = cross2D(ra_vec, n)
    rb_n = cross2D(rb_vec, n)
    inv_mass_eff = (
        a.inv_mass + b.inv_mass +
        (ra_n * ra_n) * a.inv_inertia +
        (rb_n * rb_n) * b.inv_inertia
    )

    jn = -v_rel_n / (inv_mass_eff + 1e-12)
    impulse_n = jn * n

    a.apply_impulse(-impulse_n, world_point=contact)
    b.apply_impulse( impulse_n, world_point=contact)

    # from here on, newly added friction
    mu = 0.5  


    vA = a.velocity + np.array(
        [-a.angular_velocity * ra_vec[1],
          a.angular_velocity * ra_vec[0]]
    )
    vB = b.velocity + np.array(
        [-b.angular_velocity * rb_vec[1],
          b.angular_velocity * rb_vec[0]]
    )
    v_rel = vB - vA

    v_rel_n2 = np.dot(v_rel, n)
    vt = v_rel - v_rel_n2 * n
    vt_len = np.linalg.norm(vt)
    if vt_len < 1e-6:
        return 

    t = vt / vt_len 
    ra_t = cross2D(ra_vec, t)
    rb_t = cross2D(rb_vec, t)
    inv_mass_eff_t = (
        a.inv_mass + b.inv_mass +
        (ra_t * ra_t) * a.inv_inertia +
        (rb_t * rb_t) * b.inv_inertia
    )

    v_rel_t = np.dot(v_rel, t)
    jt = -v_rel_t / (inv_mass_eff_t + 1e-12)

    max_jt = mu * jn
    jt = np.clip(jt, -max_jt, max_jt)

    impulse_t = jt * t
    a.apply_impulse(-impulse_t, world_point=contact)
    b.apply_impulse( impulse_t, world_point=contact)
    # lin_damp = 0.9  
    # ang_damp = 0.9   

    # a.velocity *= lin_damp
    # b.velocity *= lin_damp
    # a.angular_velocity *= ang_damp
    # b.angular_velocity *= ang_damp


def pick_body(bodies, world_point, max_dist=1.5):
    """
    """
    best = None
    best_d2 = (max_dist ** 2)
    wp = np.asarray(world_point, dtype=float)

    for body in bodies:
        d2 = float(np.sum((body.position - wp) ** 2))
        if d2 < best_d2:
            best_d2 = d2
            best = body
    return best


# main demo

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
    gravity = 9.8
    gravity_on = True

    # create bodies
    shape_big = PolygonShape(make_box(2.0, 1.0))
    shape_small = PolygonShape(make_box(1.2, 0.6))

    bodies = []

    rb1 = RigidBody2D(
        shape=shape_big,
        mass=2.0,
        position=(0.0, 2.2),
        angle=0.0,
        velocity=(0.0, 0.0),
        angular_velocity=0.0,
    )
    bodies.append(rb1)

    rb2 = RigidBody2D(
        shape=shape_small,
        mass=1.5,
        position=(1.8, 3.2),
        angle=0.3,
        velocity=(-0.2, 0.0),
        angular_velocity=0.5,
    )
    bodies.append(rb2)

    rb3 = RigidBody2D(
        shape=shape_small,
        mass=1.0,
        position=(-2.0, 3.0),
        angle=-0.4,
        velocity=(0.3, 0.0),
        angular_velocity=-0.3,
    )
    bodies.append(rb3)

    ground_y = 0.0

    if HAS_UI:
        pause_btn = Button(12, 12, 110, 40, "Pause", (85, 150, 110))
        reset_btn = Button(132, 12, 110, 40, "Reset", (150, 95, 95))
        step_btn  = Button(252, 12, 110, 40, "Step",  (95, 95, 150))
        grav_btn  = Button(372, 12, 140, 40, "Gravity: ON", (120, 120, 90))
        sub_slider = Slider(12, 70, 240, 1, 8, 2, "Substeps")  # 1~8 substeps
        info = InfoPanel(SCREEN_W - 260, 12)

    paused = False
    elapsed = 0.0
    grabbed_body = None      
    mouse_world = np.zeros(2) 

    def reset_state():
        bodies.clear()

        rb1 = RigidBody2D(
            shape=shape_big,
            mass=2.0,
            position=(0.0, 2.2),
            angle=0.0,
            velocity=(0.0, 0.0),
            angular_velocity=0.0,
        )
        bodies.append(rb1)

        rb2 = RigidBody2D(
            shape=shape_small,
            mass=1.5,
            position=(1.8, 3.2),
            angle=0.3,
            velocity=(-0.2, 0.0),
            angular_velocity=0.5,
        )
        bodies.append(rb2)

        rb3 = RigidBody2D(
            shape=shape_small,
            mass=1.0,
            position=(-2.0, 3.0),
            angle=-0.4,
            velocity=(0.3, 0.0),
            angular_velocity=-0.3,
        )
        bodies.append(rb3)

    running = True
    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False

            if e.type == pygame.MOUSEBUTTONDOWN and e.button == 1:
                mx, my = e.pos
                mouse_world = np.array(
                    [(mx - ORIGIN[0]) / PPM,
                     -(my - ORIGIN[1]) / PPM],
                    dtype=float
                )
                target = pick_body(bodies, mouse_world)
                if target is not None:
                    grabbed_body = target

            if e.type == pygame.MOUSEMOTION:
                mx, my = e.pos
                mouse_world = np.array(
                    [(mx - ORIGIN[0]) / PPM,
                     -(my - ORIGIN[1]) / PPM],
                    dtype=float
                )

            if e.type == pygame.MOUSEBUTTONUP and e.button == 1:
                if grabbed_body is not None:
                    grabbed_body.velocity[:] = 0.0
                    grabbed_body.angular_velocity = 0.0
                grabbed_body = None

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
                        for body in bodies:
                            body.apply_force([0.0, -gravity * body.mass])
                    for body in bodies:
                        body.integrate(dt)
                    for body in bodies:
                        collide_with_ground(body, ground_y)
                    for i in range(len(bodies)):
                        for j in range(i + 1, len(bodies)):
                            collide_bodies(bodies[i], bodies[j])
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
            dt_sub = dt / substeps
            for _ in range(substeps):

                for body in bodies:
                    if body is grabbed_body:
                        k_mouse = 200.0   
                        d_mouse = 20.0   
                        to_mouse = mouse_world - body.position
                        mouse_force = k_mouse * to_mouse - d_mouse * body.velocity
                        body.apply_force(mouse_force)
                    else:
                        if gravity_on:
                            body.apply_force([0.0, -gravity * body.mass])

                for body in bodies:
                    body.integrate(dt_sub)

                for body in bodies:
                    collide_with_ground(body, ground_y)

                for i in range(len(bodies)):
                    for j in range(i + 1, len(bodies)):
                        collide_bodies(bodies[i], bodies[j])

                elapsed += dt_sub


        for body in bodies:
            body.angular_velocity *= 0.995

        # ---------- rendering ----------
        screen.fill(BACKGROUND)

        # ground line
        y0 = world_to_screen((0, ground_y), ORIGIN, PPM)[1]
        pygame.draw.line(screen, (100, 100, 100),
                         (0, y0), (SCREEN_W, y0), 2)

        # draw all bodies
        colors = [(0, 140, 255), (255, 140, 0), (100, 220, 120)]
        for idx, body in enumerate(bodies):
            draw_polygon(screen, body.world_vertices(), ORIGIN, PPM,
                         color=colors[idx % len(colors)])

        # info text (just show first body)
        main_body = bodies[0]
        info_lines = [
            f"t={elapsed:.3f}s",
            f"pos=({main_body.position[0]:.3f},{main_body.position[1]:.3f}) m",
            f"vel=({main_body.velocity[0]:.3f},{main_body.velocity[1]:.3f}) m/s",
            f"theta={main_body.angle:.3f} rad, omega={main_body.angular_velocity:.3f} rad/s",
            f"gravity={'ON' if gravity_on else 'OFF'}  g={gravity:.1f} m/s^2",
            "LMB: impulse at mouse point (Ctrl: spin only)",
            "SPACE: pause/play, R: reset, G: toggle gravity",
        ]

        if HAS_UI:
            info.draw(screen, elapsed, main_body.position, main_body.velocity, paused)
            y = 12 + 25 * 4
            txt = font.render(
                f"θ={main_body.angle:.3f} rad, ω={main_body.angular_velocity:.3f} rad/s",
                True, (200, 200, 200))
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
