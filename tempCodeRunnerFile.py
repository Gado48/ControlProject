import pygame

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((400, 300))
pygame.display.set_caption("Robot Control")
clock = pygame.time.Clock()  # Use Pygame Clock for frame rate control

# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_w:
                print("Moving Forward")
            elif event.key == pygame.K_s:
                print("Moving Backward")
            elif event.key == pygame.K_d:
                print("Turning Right")
            elif event.key == pygame.K_a:
                print("Turning Left")

    # Optional: Clear the screen and update display
    screen.fill((0, 0, 0))
    pygame.display.flip()

    # Regulate frame rate
    clock.tick(60)  # Limit to 60 frames per second

pygame.quit()
