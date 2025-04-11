import pygame
import sys

# Initialize Pygame
pygame.init()
pygame.joystick.init()

# Configuration
DEAD_ZONE = 0.2    # Adjust this value (0.1-0.3 recommended)
PROBLEM_AXIS = 2   # Axis number showing auto-clicks

if pygame.joystick.get_count() == 0:
    print("No joysticks detected!")
    sys.exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Detected joystick: {joystick.get_name()}")
print(f"Number of axes: {joystick.get_numaxes()}")
print(f"Number of buttons: {joystick.get_numbuttons()}")
print(f"Number of hats: {joystick.get_numhats()}")
print("\nPress buttons or move controls to test (Ctrl+C to exit)...")

# Create minimal window
screen = pygame.display.set_mode((300, 200))
pygame.display.set_caption("Joystick Debug")

try:
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()
                
            # Handle axis motion with dead zone for problem axis
            elif event.type == pygame.JOYAXISMOTION:
                if event.axis == PROBLEM_AXIS:
                    if abs(event.value) > DEAD_ZONE:
                        print(f"Axis {event.axis} (filtered): {event.value:.3f}")
                else:
                    print(f"Axis {event.axis}: {event.value:.3f}")
            
            # Button press detection
            elif event.type == pygame.JOYBUTTONDOWN:
                print(f"Button {event.button} PRESSED")
            
            elif event.type == pygame.JOYBUTTONUP:
                print(f"Button {event.button} RELEASED")
            
            # Hat (D-pad) detection
            elif event.type == pygame.JOYHATMOTION:
                print(f"Hat {event.hat} moved to {event.value}")

except KeyboardInterrupt:
    print("\nExiting...")
finally:
    pygame.quit()