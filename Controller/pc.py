import os
import pygame
import socket

# Allow joystick events in background
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

ESP32_IP = "192.168.1.128"   # Replace with your ESP32's IP
ESP32_PORT = 4210

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise RuntimeError("No controller detected!")

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Connected to controller: {joystick.get_name()}")

# Track last states
last_axes = [0.0] * joystick.get_numaxes()
last_buttons = [0] * joystick.get_numbuttons()
last_hats = [(0,0)] * joystick.get_numhats()

# --- UI Settings ---
screen = pygame.display.set_mode((800, 700)) # Increased window size
pygame.display.set_caption("PS5 Controller")
font = pygame.font.SysFont("Consolas", 16)
clock = pygame.time.Clock()

COLUMN_SPACING = 400
LEFT_COLUMN_X = 50
RIGHT_COLUMN_X = LEFT_COLUMN_X + COLUMN_SPACING
Y_START = 40
Y_SPACING = 25

running = True

while running:
    pygame.event.pump()
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    screen.fill((20,20,24))

    # === AXES (Left Column) ===
    for i in range(joystick.get_numaxes()):
        val = joystick.get_axis(i)
        if abs(val - last_axes[i]) > 0.01:
            last_axes[i] = val
            msg = f"AXIS {i} {val:.3f}"
            sock.sendto(msg.encode(), (ESP32_IP, ESP32_PORT))

        # Visual representation
        bar_len = int((val + 1.0) * 100) # Scale bar length
        bar_y = Y_START + i * Y_SPACING
        pygame.draw.rect(screen, (50,50,50), (LEFT_COLUMN_X, bar_y, 200, 18)) # Background for bar
        pygame.draw.rect(screen, (0,180,80), (LEFT_COLUMN_X, bar_y, bar_len, 18))
        text = font.render(f"Axis {i}: {val:.3f}", True, (230,230,230))
        screen.blit(text, (LEFT_COLUMN_X + 220, bar_y))

    # === BUTTONS (Right Column) ===
    for i in range(joystick.get_numbuttons()):
        state = joystick.get_button(i)
        if state != last_buttons[i]:
            last_buttons[i] = state
            msg = f"BUTTON {i} {state}"
            sock.sendto(msg.encode(), (ESP32_IP, ESP32_PORT))

        # Visual representation
        button_y = Y_START + i * Y_SPACING +150  # Offset buttons down
        color = (220,60,60) if state else (80,80,90)
        pygame.draw.rect(screen, color, (RIGHT_COLUMN_X, button_y, 100, 18))
        text = font.render(f"Button {i}", True, (230,230,230))
        screen.blit(text, (RIGHT_COLUMN_X + 150, button_y))


    # === HAT (D-pad) (Bottom of Left Column) ===
    hat_y_pos = Y_START + joystick.get_numaxes() * Y_SPACING + 40 # Position after axes
    for i in range(joystick.get_numhats()):
        hat = joystick.get_hat(i)
        if hat != last_hats[i]:
            last_hats[i] = hat
            msg = f"HAT {i} {hat[0]} {hat[1]}"
            sock.sendto(msg.encode(), (ESP32_IP, ESP32_PORT))
        text = font.render(f"D-pad {i}: {hat}", True, (230,230,230))
        screen.blit(text, (LEFT_COLUMN_X, hat_y_pos + i * Y_SPACING))

    pygame.display.flip()
    clock.tick(60)
