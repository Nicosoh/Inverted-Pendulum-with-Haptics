import pygame
import numpy as np

class ScreenManager:
    def __init__(self, network):
        self.screens = {}  # Dictionary to store screens
        self.current_screen = None
        self.network = network

    def add_screen(self, name, screen):
        """Register a screen with a name."""
        self.screens[name] = screen

    def set_screen(self, name):
        """Switch to the specified screen."""
        if name in self.screens:
            self.current_screen = self.screens[name]
        if name == "game":
            self.network.send_force(np.array([0,0]))

    def handle_events(self, events):
        """Pass events to the current screen."""
        if self.current_screen:
            self.current_screen.handle_events(events)

    def draw(self, screen, *args):
        """Draw the current screen, passing necessary arguments."""
        if self.current_screen:
            self.current_screen.draw(screen, *args)


class StartPage:
    def __init__(self, screen_manager):
        self.font = pygame.font.Font(None, 36)
        self.mpc_enabled = True  # Default state of MPC
        self.screen_manager = screen_manager  # Store reference
    def handle_events(self, events):
        for event in events:
            if event.type == pygame.QUIT:
                return False  # Stop the game
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_e:
                    self.screen_manager.set_screen("game")  # Start game
                elif event.key == pygame.K_m:
                    self.mpc_enabled = not self.mpc_enabled  # Toggle MPC
        return True  # Continue running

    def draw(self, screen):
        screen.fill((0, 0, 0))  # Background color
        screen_width, screen_height = screen.get_size()

        # Render text
        title = self.font.render("Welcome to the Inverted Pendulum", True, (255, 255, 255))
        instruction1 = self.font.render("Press 'E' to Start the Game", True, (200, 200, 200))
        instruction2 = self.font.render("Press 'M' to Toggle MPC On/Off", True, (200, 200, 200))
        mpc_status = self.font.render(f"MPC: {'Enabled' if self.mpc_enabled else 'Disabled'}", True, (255, 255, 0))

        # Get centered positions
        title_rect = title.get_rect(center=(screen_width // 2, 100))
        instruction1_rect = instruction1.get_rect(center=(screen_width // 2, 200))
        instruction2_rect = instruction2.get_rect(center=(screen_width // 2, 250))
        mpc_status_rect = mpc_status.get_rect(center=(screen_width // 2, 300))

        # Draw text
        screen.blit(title, title_rect)
        screen.blit(instruction1, instruction1_rect)
        screen.blit(instruction2, instruction2_rect)
        screen.blit(mpc_status, mpc_status_rect)


class GameScreen:
    def __init__(self, screen_manager, scale_factor, cart_y_coord):
        self.font = pygame.font.Font(None, 36)
        self.scale_factor = scale_factor
        self.cart_y_coord = cart_y_coord
        self.screen_manager = screen_manager

    def handle_events(self, events):
        for event in events:
            if event.type == pygame.QUIT:
                return False  # Stop the game
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:  # Press Q to quit
                    pygame.quit()
        return True  # Continue running

    def draw(self, screen, x1, theta, score, high_score, fps, time_left):
        """ Draws the cart-pendulum system and game text """

        # Convert to pixels
        rod_length = self.scale_factor  
        cart_width = 0.3 * self.scale_factor
        cart_height = 0.1 * self.scale_factor

        x1_pixels = x1 * self.scale_factor + screen.get_width() // 2
        pendulum_x = x1_pixels - rod_length * np.sin(theta)
        pendulum_y = self.cart_y_coord + rod_length * np.cos(theta)

        # Clear screen
        screen.fill((255, 255, 255))

        # Draw pendulum
        pygame.draw.line(screen, (0, 0, 0), (x1_pixels, self.cart_y_coord), (pendulum_x, pendulum_y), 3)
        pygame.draw.circle(screen, (255, 0, 0), (int(pendulum_x), int(pendulum_y)), 10)

        # Draw cart
        pygame.draw.rect(screen, (0, 0, 255), (x1_pixels - cart_width / 2, self.cart_y_coord, cart_width, cart_height))

        # Display text
        score_text = self.font.render(f"Score: {score:.0f}", True, (0, 0, 0))
        high_score_text = self.font.render(f"High Score: {high_score:.0f}", True, (0, 0, 0))
        fps_text = self.font.render(f"FPS: {fps:.1f}", True, (0, 0, 0))
        timer_text = self.font.render(f"Time Left: {time_left:.1f} s", True, (255, 0, 0))

        screen.blit(score_text, (20, 20))
        screen.blit(high_score_text, (20, 50))
        screen.blit(fps_text, (20, 80))
        screen.blit(timer_text, (screen.get_width() - 200, 20))

class GameOverScreen:
    def __init__(self, screen_manager, reset_callback):
        self.font = pygame.font.Font(None, 48)
        self.screen_manager = screen_manager
        self.reset_callback = reset_callback 

    def handle_events(self, events):
        for event in events:
            if event.type == pygame.QUIT:
                return False  # Quit game
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:  # Press R to restart
                    self.reset_callback()  # Call the reset function
                    self.screen_manager.set_screen("start")  # Go back to start menu
                elif event.key == pygame.K_q:  # Press Q to quit
                    pygame.quit()
                    return False  
        return True  

    def draw(self, screen, score, high_score):
        screen.fill((0, 0, 0))  # Black background
        screen_width, screen_height = screen.get_size()

        # Render text
        game_over_text = self.font.render("Game Over!", True, (255, 0, 0))
        restart_text = self.font.render("Press 'R' to Restart", True, (200, 200, 200))
        quit_text = self.font.render("Press 'Q' to Quit", True, (200, 200, 200))
        score_text = self.font.render(f"Score: {score:.0f}", True, (200, 200, 200))
        high_score_text = self.font.render(f"High Score: {high_score:.0f}", True, (200, 200, 200))

        # Get centered positions
        game_over_rect = game_over_text.get_rect(center=(screen_width // 2, 150))
        restart_rect = restart_text.get_rect(center=(screen_width // 2, 200))
        quit_rect = quit_text.get_rect(center=(screen_width // 2, 250))

        # Draw text
        screen.blit(game_over_text, game_over_rect)
        screen.blit(restart_text, restart_rect)
        screen.blit(quit_text, quit_rect)
        screen.blit(score_text, (20, 20))
        screen.blit(high_score_text, (20, 50))