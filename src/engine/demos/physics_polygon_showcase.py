import sys
import math
import numpy as np
import pygame

from engine.rigidbody2d import PolygonShape, RigidBody2D, cross2D
from engine.ui_controls import Button, Slider, InfoPanel

RESTING_VELOCITY_EPS = 0.2
TANGENTIAL_VEL_EPS = 0.05
SCREEN_W, SCREEN_H = 900, 640
BACKGROUND = (22, 24, 28)
STACKING_RELVEL_EPS = 1.0
PPM = 80.0
ORIGIN = (SCREEN_W // 2, SCREEN_H // 2 + 80)

GRAVITY = 9.8
AIR_DAMP = 0.985
ANGULAR_DAMP = 0.98
MAX_ANGULAR_SPEED = 25.0

FLOOR_Y = -2.5

RESTIUTION_DEFAULT = 0.15
FRICTION_DEFAULT = 0.35
PENETRATION_SLOP = 1e-4
POSITION_CORRECT_FACTOR = 0.4


def world_to_screen(p):
    return (int(ORIGIN[0] + p[0] * PPM),
            int(ORIGIN[1] - p[1] * PPM))


def screen_to_world(pos):
    mx, my = pos
    return np.array(
        [(mx - ORIGIN[0]) / PPM,
         -(my - ORIGIN[1]) / PPM],
        dtype=float
    )


def draw_polygon(screen, body, color=(0, 150, 255), width=0):
    verts = body.world_vertices()
    pts = [world_to_screen(v) for v in verts]
    if len(pts) >= 3:
        pygame.draw.polygon(screen, color, pts, width)
        pygame.draw.polygon(screen, (255, 255, 255), pts, 2)


def draw_floor_and_ramp_line(screen):
    y0 = world_to_screen((0.0, FLOOR_Y))[1]
    pygame.draw.line(screen, (80, 80, 80), (0, y0), (SCREEN_W, y0), 1)


def draw_drawing_preview(screen, current_points_world):
    if not current_points_world:
        return
    pts = [world_to_screen(p) for p in current_points_world]
    for pt in pts:
        pygame.draw.circle(screen, (230, 200, 120), pt, 4)
    if len(pts) >= 2:
        pygame.draw.lines(screen, (230, 200, 120), False, pts, 2)


def pick_body(bodies, world_point, max_dist=0.5):
    best = None
    best_d2 = max_dist ** 2
    wp = np.asarray(world_point, dtype=float)
    for b in bodies:
        if b.inv_mass == 0.0:
            continue
        d2 = float(np.sum((b.position - wp) ** 2))
        if d2 < best_d2:
            best_d2 = d2
            best = b
    return best


def create_polygon_body(world_points, density=1.0, vel=(0.0, 0.0),
                        mass=None, friction=FRICTION_DEFAULT,
                        restitution=RESTIUTION_DEFAULT, is_round=False):
    shape = PolygonShape(world_points)
    centroid = shape.local_centroid.copy()

    if mass is not None:
        body = RigidBody2D(
            shape=shape,
            mass=mass,
            position=centroid,
            angle=0.0,
            velocity=vel,
            angular_velocity=0.0
        )
    else:
        body = RigidBody2D(
            shape=shape,
            density=density,
            position=centroid,
            angle=0.0,
            velocity=vel,
            angular_velocity=0.0
        )

    body.friction = float(friction)
    body.restitution = float(restitution)
    body.is_round = bool(is_round)

    if not body.is_round and body.inv_inertia > 0.0:
        factor = 5.0
        body.inertia *= factor
        body.inv_inertia /= factor

    return body


def create_box(center, w, h, density=1.0, mass=None,
               friction=0.18, restitution=RESTIUTION_DEFAULT):
    cx, cy = center
    hw, hh = w / 2.0, h / 2.0
    pts = [
        [cx - hw, cy - hh],
        [cx + hw, cy - hh],
        [cx + hw, cy + hh],
        [cx - hw, cy + hh],
    ]
    return create_polygon_body(pts, density=density, mass=mass,
                               friction=friction, restitution=restitution, is_round=False)


def create_ngon(center, radius, n_sides=40, density=1.0, mass=None,
                friction=0.55, restitution=RESTIUTION_DEFAULT):
    cx, cy = center
    pts = []
    for i in range(n_sides):
        ang = 2.0 * math.pi * i / n_sides
        x = cx + radius * math.cos(ang)
        y = cy + radius * math.sin(ang)
        pts.append([x, y])
    return create_polygon_body(pts, density=density, mass=mass,
                               friction=friction, restitution=restitution, is_round=True)


def create_static_ramp(center, length, thickness, angle_deg,
                       friction=0.8):
    cx, cy = center
    theta = math.radians(angle_deg)

    d = np.array([math.cos(theta), math.sin(theta)], dtype=float)
    n = np.array([-d[1], d[0]], dtype=float)

    hl = length / 2.0
    ht = thickness / 2.0

    p1 = [cx - hl * d[0] - ht * n[0], cy - hl * d[1] - ht * n[1]]
    p2 = [cx + hl * d[0] - ht * n[0], cy + hl * d[1] - ht * n[1]]
    p3 = [cx + hl * d[0] + ht * n[0], cy + hl * d[1] + ht * n[1]]
    p4 = [cx - hl * d[0] + ht * n[0], cy - hl * d[1] + ht * n[1]]

    return create_polygon_body([p1, p2, p3, p4], mass=0.0,
                               friction=friction, restitution=0.0)


def create_static_floor(width=40.0, thickness=0.5, friction=0.8):
    cy = FLOOR_Y - thickness / 2.0
    cx = 0.0
    return create_box(center=(cx, cy), w=width, h=thickness,
                      mass=0.0, friction=friction, restitution=0.0)


def sat_overlap_on_axis(body_a, body_b, axis):
    axis = np.asarray(axis, dtype=float)
    axis /= (np.linalg.norm(axis) + 1e-12)

    minA, maxA = body_a.project_onto_axis(axis)
    minB, maxB = body_b.project_onto_axis(axis)

    overlap = min(maxA, maxB) - max(minA, minB)
    return overlap


def polygon_polygon_sat(body_a, body_b):
    if body_a.inv_mass > 0.0 and body_b.inv_mass == 0.0:
        axes = body_b.world_normals()
    elif body_b.inv_mass > 0.0 and body_a.inv_mass == 0.0:
        axes = body_a.world_normals()
    else:
        axes = np.vstack((body_a.world_normals(), body_b.world_normals()))

    smallest_overlap = float("inf")
    smallest_axis = None

    for axis in axes:
        overlap = sat_overlap_on_axis(body_a, body_b, axis)
        if overlap <= 0.0:
            return None

        if overlap < smallest_overlap:
            smallest_overlap = overlap
            d = body_b.position - body_a.position
            if np.dot(d, axis) < 0.0:
                axis = -axis
            smallest_axis = axis.copy()

    return smallest_axis, smallest_overlap


def contact_point_from_normal(body_a, body_b, normal):
    n = np.asarray(normal, dtype=float)
    n /= (np.linalg.norm(n) + 1e-12)

    dyn = None
    if body_a.inv_mass > 0.0 and body_b.inv_mass == 0.0:
        dyn = body_a
        dir_vec = n
    elif body_b.inv_mass > 0.0 and body_a.inv_mass == 0.0:
        dyn = body_b
        dir_vec = -n
    else:
        dyn = None

    if dyn is not None:
        verts = dyn.world_vertices()
        proj = verts @ dir_vec
        max_proj = proj.max()
        eps = 1e-4
        mask = proj >= max_proj - eps
        contact_pts = verts[mask]
        return contact_pts.mean(axis=0)

    pa = body_a.support_point(-n)
    pb = body_b.support_point(n)
    return 0.5 * (pa + pb)


def cross_z_v(z, v):
    return np.array([-z * v[1], z * v[0]], dtype=float)


def get_body_friction(body):
    return getattr(body, "friction", FRICTION_DEFAULT)


def get_body_restitution(body):
    return getattr(body, "restitution", RESTIUTION_DEFAULT)


def solve_velocity_contact(body_a, body_b, contact_point, normal):
    n = np.asarray(normal, dtype=float)
    n /= (np.linalg.norm(n) + 1e-12)

    ra = contact_point - body_a.position
    rb = contact_point - body_b.position

    invMassA, invMassB = body_a.inv_mass, body_b.inv_mass
    invIA, invIB = body_a.inv_inertia, body_b.inv_inertia
    if invMassA + invMassB == 0.0:
        return

    is_round_a = getattr(body_a, "is_round", False)
    is_round_b = getattr(body_b, "is_round", False)
    round_pair = is_round_a or is_round_b
    box_pair = (not is_round_a) and (not is_round_b)
    dynamic_dynamic = (invMassA > 0.0 and invMassB > 0.0)

    velA = body_a.velocity
    velB = body_b.velocity
    rv = velB - velA
    vel_norm = np.dot(rv, n)

    if (dynamic_dynamic and box_pair and
            abs(vel_norm) < RESTING_VELOCITY_EPS * 2.0 and
            np.linalg.norm(rv) < STACKING_RELVEL_EPS):

        denom_n = invMassA + invMassB
        if denom_n > 0.0:
            restitution = min(get_body_restitution(body_a),
                              get_body_restitution(body_b))
            restitution = 0.0
            vel_norm = 0.0

            jn = -(1.0 + restitution) * vel_norm / denom_n
            if jn < 0.0:
                jn = 0.0

            impulse_n = jn * n
            body_a.velocity -= impulse_n * invMassA
            body_b.velocity += impulse_n * invMassB

            rv = body_b.velocity - body_a.velocity
            vt = rv - np.dot(rv, n) * n
            t_len = np.linalg.norm(vt)
            if t_len > TANGENTIAL_VEL_EPS:
                t = vt / t_len
                denom_t = invMassA + invMassB
                if denom_t > 0.0:
                    mu = min(get_body_friction(body_a),
                             get_body_friction(body_b)) * 0.3
                    jt = -np.dot(rv, t) / denom_t
                    max_jt = mu * jn
                    jt = np.clip(jt, -max_jt, max_jt)
                    impulse_t = jt * t
                    body_a.velocity -= impulse_t * invMassA
                    body_b.velocity += impulse_t * invMassB

        return

    velA = body_a.velocity + cross_z_v(body_a.angular_velocity, ra)
    velB = body_b.velocity + cross_z_v(body_b.angular_velocity, rb)
    rv = velB - velA

    vel_norm = np.dot(rv, n)
    if vel_norm > 0.0:
        return

    ra_cn = cross2D(ra, n)
    rb_cn = cross2D(rb, n)
    denom_n = invMassA + invMassB + (ra_cn * ra_cn) * invIA + (rb_cn * rb_cn) * invIB
    if denom_n <= 0.0:
        return

    restitution = min(get_body_restitution(body_a), get_body_restitution(body_b))
    if abs(vel_norm) < RESTING_VELOCITY_EPS:
        restitution = 0.0
        vel_norm = 0.0

    jn = -(1.0 + restitution) * vel_norm / denom_n
    if jn < 0.0:
        jn = 0.0

    impulse_n = jn * n
    body_a.apply_impulse(-impulse_n, contact_point)
    body_b.apply_impulse(impulse_n, contact_point)

    velA = body_a.velocity + cross_z_v(body_a.angular_velocity, ra)
    velB = body_b.velocity + cross_z_v(body_b.angular_velocity, rb)
    rv = velB - velA

    vt = rv - np.dot(rv, n) * n
    t_len = np.linalg.norm(vt)
    if t_len < TANGENTIAL_VEL_EPS:
        return

    t = vt / t_len

    ra_ct = cross2D(ra, t)
    rb_ct = cross2D(rb, t)
    denom_t = invMassA + invMassB + (ra_ct * ra_ct) * invIA + (rb_ct * rb_ct) * invIB
    if denom_t <= 0.0:
        return

    mu = min(get_body_friction(body_a), get_body_friction(body_b))
    if not round_pair:
        mu *= 0.1

    jt = -np.dot(rv, t) / denom_t
    max_jt = mu * jn
    jt = np.clip(jt, -max_jt, max_jt)
    impulse_t = jt * t

    if round_pair:
        body_a.apply_impulse(-impulse_t, contact_point)
        body_b.apply_impulse(impulse_t, contact_point)
    else:
        body_a.apply_impulse(-impulse_t, contact_point)
        body_b.apply_impulse(impulse_t, contact_point)


def positional_correction(body_a, body_b, normal, penetration):
    if penetration <= PENETRATION_SLOP:
        return

    n = np.asarray(normal, dtype=float)
    n /= (np.linalg.norm(n) + 1e-12)

    invMassA, invMassB = body_a.inv_mass, body_b.inv_mass
    total_inv_mass = invMassA + invMassB
    if total_inv_mass <= 0.0:
        return

    depth = (penetration - PENETRATION_SLOP) * POSITION_CORRECT_FACTOR

    if invMassA == 0.0 and invMassB > 0.0:
        body_b.position += depth * n
    elif invMassB == 0.0 and invMassA > 0.0:
        body_a.position -= depth * n
    else:
        correction = depth * n / total_inv_mass
        body_a.position -= correction * invMassA
        body_b.position += correction * invMassB


def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
    pygame.display.set_caption("Polygon Collision Demo")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 24)

    BASE_DT = 1.0 / 120.0

    bodies = []

    floor = create_static_floor(width=40.0, thickness=0.5, friction=0.8)
    ramp = create_static_ramp(
        center=(-0.5, -0.6),
        length=3.2,
        thickness=0.25,
        angle_deg=25,
        friction=0.7,
    )
    bodies.extend([floor, ramp])

    pause_btn = Button(12, 12, 90, 32, "Pause", (85, 150, 110))
    reset_btn = Button(112, 12, 90, 32, "Reset", (150, 95, 95))
    draw_btn = Button(212, 12, 130, 32, "Draw Polygon", (95, 140, 200))
    box_btn = Button(352, 12, 110, 32, "Add Box", (140, 120, 90))
    circle_btn = Button(472, 12, 140, 32, "Add Circle", (120, 95, 150))
    gravity_btn = Button(622, 12, 150, 32, "Gravity: ON", (120, 120, 90))

    sub_slider = Slider(12, 60, 240, 1, 8, 4, "Speed")
    info_panel = InfoPanel(SCREEN_W - 250, 12)

    paused = False
    gravity_on = True
    elapsed = 0.0

    main_body = None

    draw_mode = False
    current_points_world = []

    grabbed_body = None
    mouse_world = np.zeros(2, dtype=float)

    def reset_world():
        bodies.clear()
        floor = create_static_floor(width=40.0, thickness=0.5, friction=0.8)
        ramp = create_static_ramp(center=(-1.2, -0.8),
                                  length=5.0, thickness=0.25,
                                  angle_deg=25, friction=0.7)
        bodies.extend([floor, ramp])

    running = True
    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False

            if e.type in (pygame.MOUSEMOTION,
                          pygame.MOUSEBUTTONDOWN,
                          pygame.MOUSEBUTTONUP):
                mouse_world = screen_to_world(e.pos)
                sub_slider.handle_event(e)

            if e.type == pygame.MOUSEBUTTONDOWN:
                if e.button == 1:
                    if draw_mode:
                        current_points_world.append(mouse_world.copy())
                    else:
                        if not any(btn.rect.collidepoint(e.pos) for btn in
                                   [pause_btn, reset_btn, draw_btn,
                                    box_btn, circle_btn, gravity_btn]):
                            grabbed_body = pick_body(bodies, mouse_world,
                                                     max_dist=0.6)
                if e.button == 3 and draw_mode:
                    if len(current_points_world) >= 3:
                        body = create_polygon_body(current_points_world,
                                                   density=1.0,
                                                   friction=0.2, is_round=False)
                        bodies.append(body)
                        main_body = body
                    current_points_world = []
                    draw_mode = False
                    draw_btn.text = "Draw Polygon"

            if e.type == pygame.MOUSEBUTTONUP and e.button == 1:
                grabbed_body = None

            if e.type == pygame.KEYDOWN:
                if e.key == pygame.K_SPACE:
                    paused = not paused
                    pause_btn.text = "Play" if paused else "Pause"
                elif e.key == pygame.K_r:
                    reset_world()
                    elapsed = 0.0
                    paused = False
                    pause_btn.text = "Pause"
                    main_body = None
                    current_points_world = []
                    draw_mode = False
                    draw_btn.text = "Draw Polygon"
                elif e.key == pygame.K_g:
                    gravity_on = not gravity_on
                    gravity_btn.text = f"Gravity: {'ON' if gravity_on else 'OFF'}"

            if pause_btn.handle_event(e):
                paused = not paused
                pause_btn.text = "Play" if paused else "Pause"

            if reset_btn.handle_event(e):
                reset_world()
                elapsed = 0.0
                paused = False
                pause_btn.text = "Pause"
                main_body = None
                current_points_world = []
                draw_mode = False
                draw_btn.text = "Draw Polygon"

            if draw_btn.handle_event(e):
                draw_mode = not draw_mode
                current_points_world = []
                draw_btn.text = "Finish Drawing" if draw_mode else "Draw Polygon"

            if box_btn.handle_event(e):
                body = create_box(center=(0.5, 1.8),
                                  w=1.0, h=0.5,
                                  density=1.0, friction=0.18)
                bodies.append(body)
                main_body = body

            if circle_btn.handle_event(e):
                body = create_ngon(center=(-2.0, 1.5),
                                   radius=0.35,
                                   n_sides=40,
                                   density=1.0,
                                   friction=0.55)
                bodies.append(body)
                main_body = body

            if gravity_btn.handle_event(e):
                gravity_on = not gravity_on
                gravity_btn.text = f"Gravity: {'ON' if gravity_on else 'OFF'}"

        if not paused:
            speed_raw = sub_slider.get_value()
            speed_factor = speed_raw / 2.0
            dt = BASE_DT * speed_factor

            for body in bodies:
                if body is grabbed_body and body.inv_mass > 0.0:
                    body.position = mouse_world.copy()
                    body.velocity[:] = 0.0
                    body.angular_velocity = 0.0
                    body.clear_forces()
                else:
                    if gravity_on and body.inv_mass > 0.0:
                        body.apply_force([0.0, -GRAVITY * body.mass])
                    body.integrate(dt)
                    body.velocity *= AIR_DAMP
                    body.angular_velocity *= ANGULAR_DAMP
                    if abs(body.angular_velocity) > MAX_ANGULAR_SPEED:
                        body.angular_velocity = math.copysign(
                            MAX_ANGULAR_SPEED, body.angular_velocity
                        )

            contacts = []
            n_bodies = len(bodies)
            for i in range(n_bodies):
                for j in range(i + 1, n_bodies):
                    a = bodies[i]
                    b = bodies[j]
                    info = polygon_polygon_sat(a, b)
                    if info is None:
                        continue
                    normal, penetration = info
                    contact = contact_point_from_normal(a, b, normal)
                    contacts.append((a, b, contact, normal, penetration))

            for _ in range(4):
                for a, b, cp, n, pen in contacts:
                    solve_velocity_contact(a, b, cp, n)

            for a, b, cp, n, pen in contacts:
                positional_correction(a, b, n, pen)

            for body in bodies:
                if body.inv_mass == 0.0:
                    continue
                if np.linalg.norm(body.velocity) < 0.03 and abs(body.angular_velocity) < 0.03:
                    body.velocity[:] = 0.0
                    body.angular_velocity = 0.0

            elapsed += dt

        screen.fill(BACKGROUND)

        draw_floor_and_ramp_line(screen)

        colors = [
            (160, 160, 160),
            (130, 130, 130),
            (0, 140, 255),
            (255, 140, 0),
            (100, 220, 120),
            (200, 160, 90),
            (160, 120, 200),
        ]
        for i, body in enumerate(bodies):
            draw_polygon(screen, body, color=colors[i % len(colors)])

        if draw_mode:
            draw_drawing_preview(screen, current_points_world)
            msg = "Drawing: Left click = add point, Right click = finish"
            tip = font.render(msg, True, (230, 230, 160))
            screen.blit(tip, (12, SCREEN_H - 30))

        if main_body is not None:
            info_panel.draw(screen, elapsed,
                            main_body.position, main_body.velocity, paused)

        sub_slider.draw(screen)
        pause_btn.draw(screen)
        reset_btn.draw(screen)
        draw_btn.draw(screen)
        box_btn.draw(screen)
        circle_btn.draw(screen)
        gravity_btn.draw(screen)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
