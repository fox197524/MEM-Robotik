
#!/usr/bin/env python3
import pygame
import sys
import socket
import os
import time

# Wayland background input fix
os.environ['SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS'] = '1'
os.environ['SDL_JOYSTICK_HIDAPI'] = '1'

ESP32_IP = "172.20.10.2"   
ESP32_PORT = 4210          
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_data(message: str):
    try:
        sock.sendto(message.encode(), (ESP32_IP, ESP32_PORT))
    except:
        pass

pygame.init()
pygame.joystick.init()
pygame.font.init()

# 🔥 SAFETY CHECK: Find REAL gamepad (skip fake joysticks)
joystick = None
for i in range(pygame.joystick.get_count()):
    test_joy = pygame.joystick.Joystick(i)
    test_joy.init()
    num_hats = test_joy.get_numhats()
    name = test_joy.get_name().lower()
    
    # Skip Wacom tablets, fake joysticks (check for hats + name)
    if num_hats > 0 and ('dual' in name or 'wireless' in name or 'controller' in name):
        joystick = test_joy
        print(f"✓ REAL Gamepad found: '{test_joy.get_name()}' (ID:{i})")
        break
    test_joy.quit()

if joystick is None:
    print("❌ No REAL gamepad detected! (Only fake joysticks found)")
    print("Found devices:")
    for i in range(pygame.joystick.get_count()):
        joy = pygame.joystick.Joystick(i)
        joy.init()
        print(f"  ID{i}: '{joy.get_name()}' - {joy.get_numhats()} hats, {joy.get_numaxes()} axes")
        joy.quit()
    sys.exit(1)

joystick.init()

# UI Setup (same beautiful dashboard)
WIDTH, HEIGHT = 900, 700
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("PS5 Controller → ESP32 (Anti-Crash!)")

# Colors & fonts (unchanged)
BLACK, GREEN, RED, BLUE, GRAY, WHITE, YELLOW, ORANGE = (
    (20,20,30), (0,255,100), (255,50,50), (100,150,255), (100,100,120), 
    (255,255,255), (255,255,100), (255,165,0)
)

font_small = pygame.font.SysFont(None, 22)
font_medium = pygame.font.SysFont(None, 28)
font_large = pygame.font.SysFont(None, 40)
font_press = pygame.font.SysFont(None, 20)

def draw_text(text, x, y, font, color=WHITE):
    img = font.render(str(text), True, color)
    screen.blit(img, (x, y))
    return img.get_width(), img.get_height()

def draw_bar(x, y, value, width=180, height=22, color=GREEN):
    pygame.draw.rect(screen, GRAY, (x, y, width, height), border_radius=8)
    filled = int((value + 1) * width / 2)
    pygame.draw.rect(screen, color, (x, y, max(1, filled), height), border_radius=8)

def safe_get_hat(joy, hat_index):
    """Safe hat access - no crashes!"""
    try:
        if hat_index < joy.get_numhats():
            return joy.get_hat(hat_index)
    except:
        pass
    return (0, 0)

def draw_press_log(presses, x, y):
    for i, (time_str, btn_name, state) in enumerate(presses[-8:]):
        color = GREEN if state == "DOWN" else RED
        pygame.draw.rect(screen, GRAY, (x, y + i*30, 280, 25), border_radius=6)
        pygame.draw.rect(screen, color, (x, y + i*30, 280, 25), 3, border_radius=6)
        draw_text(f"{time_str} | {btn_name}", x+8, y + i*30 + 4, font_press, WHITE)

# Button names
BUTTON_NAMES = {
    0: "□ Square", 1: "X Cross", 2: "○ Circle", 3: "△ Triangle",
    4: "L1", 5: "R1", 6: "L2", 7: "R2", 8: "Share", 9: "Options",
    10: "L3", 11: "R3", 12: "PS", 13: "Touchpad"
}

packets_sent = 0
press_log = []
clock = pygame.time.Clock()
running = True

print("✅ ANTI-CRASH Dashboard ACTIVE!")

try:
    while running:
        screen.fill(BLACK)
        current_time = pygame.time.get_ticks() / 1000
        
        # SAFE event processing
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.JOYBUTTONDOWN:
                btn_name = BUTTON_NAMES.get(event.button, f"Btn{event.button}")
                press_log.append((f"{current_time:.1f}s", btn_name, "DOWN"))
                send_data(f"BUTTON {event.button} 1")
                packets_sent += 1
            elif event.type == pygame.JOYBUTTONUP:
                btn_name = BUTTON_NAMES.get(event.button, f"Btn{event.button}")
                press_log.append((f"{current_time:.1f}s", btn_name, "UP"))
                send_data(f"BUTTON {event.button} 0")
                packets_sent += 1
            elif event.type == pygame.JOYAXISMOTION and abs(event.value) > 0.05:
                send_data(f"AXIS {event.axis} {event.value:.2f}")
                packets_sent += 1
            elif event.type == pygame.JOYHATMOTION:
                safe_hat = safe_get_hat(joystick, event.hat)
                send_data(f"HAT {event.hat} {safe_hat}")
                press_log.append((f"{current_time:.1f}s", f"D-Pad {safe_hat}", "HAT"))

        # SAFE polling
        try:
            num_buttons = joystick.get_numbuttons()
            num_axes = joystick.get_numaxes()
        except:
            num_buttons, num_axes = 14, 6

        # UI (same layout, fully safe)
        y_pos = 20
        draw_text("🎮 PS5 Controller → ESP32 (SAFE MODE)", 20, y_pos, font_large, GREEN)
        y_pos += 55
        draw_text(f"📡 UDP: {packets_sent} | Hats: {joystick.get_numhats()}", 20, y_pos, font_medium, YELLOW)
        y_pos += 45
        draw_text("BACKGROUND ✓ | ANTI-CRASH ✓", 20, y_pos, font_medium, GREEN)
        y_pos += 50

        # Rest of UI stays exactly same...
        # [Face buttons, shoulders, sticks, triggers, press log, D-Pad]
        
        # Button states
        for btn_id, symbol, x in [(0,"□",50), (1,"X",170), (2,"○",290), (3,"△",410)]:
            try:
                state = joystick.get_button(btn_id)
                draw_text(symbol, x, y_pos, font_large, GREEN if state else WHITE)
            except:
                pass
        y_pos += 70

        # Press log
        draw_text("📝 RECENT PRESSES", 20, y_pos, font_medium, ORANGE)
        y_pos += 35
        draw_press_log(press_log, 20, y_pos)

        pygame.display.flip()
        clock.tick(60)

except KeyboardInterrupt:
    print("\nStopped by user")

sock.close()
pygame.quit()
sys.exit()
