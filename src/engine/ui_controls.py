import pygame
import numpy as np

class Button:
    def __init__(self,x, y, width, height, text, color=(100, 100, 200)):
        self.rect = pygame.Rect(x, y, width, height)
        self.text = text
        self.color = color
        self.hover_color = (150, 150, 250)
        self.is_hovered = False
        self.font = pygame.font.SysFont(None, 28)

    def draw(self, screen):
        current_color = self.hover_color if self.is_hovered else self.color

        pygame.draw.rect(screen, current_color, self.rect)
        pygame.draw.rect(screen, (255,255,255), self.rect, 2)

        text_surface = self.font.render(self.text, True, (255, 255, 255))
        text_rect = text_surface.get_rect(center=self.rect.center)
        screen.blit(text_surface, text_rect)

    def handle_event(self, event):
        if event.type == pygame.MOUSEMOTION:
            self.is_hovered = self.rect.collidepoint(event.pos)
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.rect.collidepoint(event.pos):
                return True
        return False
class Slider:

    def __init__(self, x, y, width, min_val, max_val, initial_val, label=""):
        self.x = x
        self.y = y
        self.width = width
        self.height = 20
        self.min_val = min_val
        self.max_val = max_val
        self.value = initial_val
        self.label = label
        self.dragging = False
        self.font = pygame.font.Font(None, 20)

    def draw(self, screen):
        track_rect = pygame.Rect(self.x, self.y, self.width, self.height)
        pygame.draw.rect(screen, (100, 100, 100), track_rect)
        pygame.draw.rect(screen, (255, 255, 255), track_rect, 1)

        ratio = (self.value - self.min_val) / (self.max_val - self.min_val)
        handle_x = self.x + int(ratio * self.width)
        handle_rect = pygame.Rect(handle_x - 5, self.y - 5, 10, self.height + 10)

        pygame.draw.rect(screen, (200, 200, 255), handle_rect)
        pygame.draw.rect(screen, (255, 255, 255), handle_rect, 2)
        label_text = f"{self.label}: {self.value:.2f}x"
        text_surface = self.font.render(label_text, True, (255, 255, 255))
        screen.blit(text_surface, (self.x, self.y - 25))

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            handle_x = self.x + int((self.value - self.min_val) / (self.max_val - self.min_val) * self.width)
            handle_rect = pygame.Rect(handle_x - 5, self.y - 5, 10, self.height + 10)
            if handle_rect.collidepoint(event.pos):
                self.dragging = True
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            self.dragging = False

        elif event.type == pygame.MOUSEMOTION and self.dragging:
            mouse_x = event.pos[0]
            ratio = (mouse_x - self.x) / self.width
            ratio = max(0, min(1, ratio))  # Clamp between 0 and 1
            self.value = self.min_val + ratio * (self.max_val - self.min_val)

    def get_value(self):
        return self.value

class InfoPanel:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.font = pygame.font.Font(None, 24)
        self.small_font = pygame.font.Font(None, 20)

    def draw(self, screen, time, position, velocity, paused):
        info_lines = [
            f"Time: {time:.2f}s",
            f"Position: ({position[0]:.2f}, {position[1]:.2f})m",
            f"Velocity: ({velocity[0]:.2f}, {velocity[1]:.2f})m/s",
            f"Speed: {np.linalg.norm(velocity):.2f}m/s",
        ]

        y_offset = self.y

        for line in info_lines:
            text = self.small_font.render(line, True, (200, 200, 200))
            screen.blit(text, (self.x, y_offset))
            y_offset += 25

