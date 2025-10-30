import pygame
import sys
from engine.body import Body
from engine.spring import Spring
from engine.world import World


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
fixed_body = Body(mass=float('inf'), position=[0, 3], velocity=[0, 0])
moving_body = Body(mass=1.0, position=[0, 1], velocity=[0, 0])
world.add_body(fixed_body)
world.add_body(moving_body)
spring = Spring(fixed_body, moving_body, k=50.0, rest_length=1.0, damping=0)
world.add_spring(spring)

paused = False


running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                paused = not paused
                print("Paused" if paused else "Unpaused")

            if event.key == pygame.K_r:
                fixed_body.position = [0, 3]
                fixed_body.velocity = [0, 0]
                moving_body.position = [0, 1]
                moving_body.velocity = [0, 0]
                fixed_body.clear_force()
                moving_body.clear_force()
                print("Reset")
    if not paused:
        moving_body.apply_force([0, -9.8])
        world.step()
    screen.fill(BACKGROUND_COLOR)
    fixed_body_pos = physics_to_display(fixed_body.position)
    moving_body_pos = physics_to_display(moving_body.position)
    pygame.draw.line(screen, (50, 50, 50), fixed_body_pos, moving_body_pos, 2)
    pygame.draw.circle(screen, (100, 100, 100), fixed_body_pos, 10)
    pygame.draw.circle(screen, (0, 100, 255), moving_body_pos, 15)
    if paused:
        pause_text = font.render("PAUSED", True, (255, 0, 0))
        screen.blit(pause_text, (10, 10))
    pygame.display.flip()
    clock.tick(FPS)

pygame.quit()
sys.exit()