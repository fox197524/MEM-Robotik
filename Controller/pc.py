import os
import pygame
import socket

# Allow joystick events in background
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

ESP32_IP = "172.20.10.2"   # Replace with your ESP32's IP
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

screen = pygame.display.set_mode((640, 420))
pygame.display.set_caption("PS5 Controller")
font = pygame.font.SysFont("Consolas", 18)
clock = pygame.time.Clock()

running = True

while running:
    pygame.event.pump()
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    screen.fill((20,20,24))

    # === AXES ===
    for i in range(joystick.get_numaxes()):
        val = joystick.get_axis(i)
        # Only send if changed significantly
        if abs(val - last_axes[i]) > 0.01:   # tolerance to avoid noise
            last_axes[i] = val
            msg = f"AXIS {i} {val:.3f}"
            sock.sendto(msg.encode(), (ESP32_IP, ESP32_PORT))
        bar_len = int((val + 1.0) * 120)
        pygame.draw.rect(screen, (0,180,80), (40, 40 + i*28, bar_len, 18))
        text = font.render(f"Axis {i}: {val:.3f}", True, (230,230,230))
        screen.blit(text, (300, 40 + i*28))

    # === BUTTONS ===
    for i in range(joystick.get_numbuttons()):
        state = joystick.get_button(i)
        if state != last_buttons[i]:
            last_buttons[i] = state
            msg = f"BUTTON {i} {state}"
            sock.sendto(msg.encode(), (ESP32_IP, ESP32_PORT))
        color = (220,60,60) if state else (80,80,90)
        pygame.draw.circle(screen, color, (40 + i*26, 260), 10)
        text = font.render(str(i), True, (230,230,230))
        screen.blit(text, (34 + i*26, 278))

    # === HAT (D-pad) ===
    for i in range(joystick.get_numhats()):
        hat = joystick.get_hat(i)
        if hat != last_hats[i]:
            last_hats[i] = hat
            msg = f"HAT {i} {hat[0]} {hat[1]}"
            sock.sendto(msg.encode(), (ESP32_IP, ESP32_PORT))
        text = font.render(f"D-pad {i}: {hat}", True, (230,230,230))
        screen.blit(text, (40, 320 + i*24))

    pygame.display.flip()
    clock.tick(60)   # smooth UI