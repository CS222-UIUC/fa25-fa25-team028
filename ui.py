import pygame
import sys

class PhysicsUI:
    def __init__(self, width=800, height=600):
        pygame.init()
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Physics Engine")
        self.clock = pygame.time.Clock()
        self.running = True

        self.options_rect = pygame.Rect(width - 150, 10, 140, 30)

    def draw_options(self):
        pygame.draw.rect(self.screen, (200, 200, 200), self.options_rect)
        font = pygame.font.SysFont(None, 24)
        text = font.render("Options ▼", True, (0, 0, 0))
        self.screen.blit(text, (self.options_rect.x + 5, self.options_rect.y + 5))

    def run(self):
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

            self.screen.fill((255, 255, 255))
            self.draw_options()
            
            pygame.display.flip()
            self.clock.tick(60)

        pygame.quit()
        sys.exit()