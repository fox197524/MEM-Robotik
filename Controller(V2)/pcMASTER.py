
#!/usr/bin/env python3
import pygame
import sys
import socket
import os

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

if pygame.joystick.get_count() == 0:
    print("No controller detected.")
    sys.exit(1)

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"✓ DualSense '{joystick.get_name()}' connected!")

# UI Setup
WIDTH, HEIGHT = 900, 700
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("PS5 Controller → ESP32 Dashboard")

# Enable background input
pygame.event.set_grab(True)

# Colors
BLACK = (20, 20, 30)
GREEN = (0, 255, 100)
RED = (255, 50, 50)
BLUE = (100, 150, 255)
GRAY = (100, 100, 120)
WHITE = (255, 255, 255)
YELLOW = (255, 255, 100)
ORANGE = (255, 165, 0)

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

def draw_press_log(presses, x, y):
    """Draw recent button presses with timestamps"""
    for i, (time_str, btn_name, state) in enumerate(presses[-8:]):  # Last 8 presses
        color = GREEN if state == "DOWN" else RED
        pygame.draw.rect(screen, GRAY, (x, y + i*30, 280, 25), border_radius=6)
        pygame.draw.rect(screen, color, (x, y + i*30, 280, 25), 3, border_radius=6)
        draw_text(f"{time_str} | {btn_name}", x+8, y + i*30 + 4, font_press, WHITE)

# Button names for PS5 controller
BUTTON_NAMES = {
    0: "□ Square", 1: "X Cross", 2: "○ Circle", 3: "△ Triangle",
    4: "L1", 5: "R1", 6: "L2", 7: "R2", 8: "Share", 9: "Options",
    10: "L3", 11: "R3", 12: "PS", 13: "Touchpad"
}

packets_sent = 0
clock = pygame.time.Clock()
running = True

# Press log: (timestamp, button_name, state)
press_log = []

print("Dashboard ACTIVE - Button presses shown in GUI!")

try:
    while running:
        screen.fill(BLACK)
        
        current_time = pygame.time.get_ticks() / 1000
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.JOYBUTTONDOWN:
                btn_name = BUTTON_NAMES.get(event.button, f"Btn{event.button}")
                press_log.append((f"{current_time:.1f}s", btn_name, "DOWN"))
                send_data(f"BUTTON {event.button} 1")
                packets_sent += 1
                print(f"BTN {event.button} ({btn_name}) ↓")
            elif event.type == pygame.JOYBUTTONUP:
                btn_name = BUTTON_NAMES.get(event.button, f"Btn{event.button}")
                press_log.append((f"{current_time:.1f}s", btn_name, "UP"))
                send_data(f"BUTTON {event.button} 0")
                packets_sent += 1
                print(f"BTN {event.button} ({btn_name}) ↑")
            elif event.type == pygame.JOYAXISMOTION:
                if abs(event.value) > 0.05:
                    send_data(f"AXIS {event.axis} {event.value:.2f}")
                    packets_sent += 1
            elif event.type == pygame.JOYHATMOTION:
                send_data(f"HAT {event.hat} {event.value}")
                press_log.append((f"{current_time:.1f}s", f"D-Pad {event.value}", "HAT"))
                packets_sent += 1

        # UI Layout
        y_pos = 20
        
        # Header
        draw_text("🎮 PS5 DualSense → ESP32 Dashboard", 20, y_pos, font_large, GREEN)
        y_pos += 55
        draw_text(f"📡 UDP Packets: {packets_sent}", 20, y_pos, font_medium, YELLOW)
        y_pos += 45
        draw_text("BACKGROUND INPUT: ✓ ENABLED", 20, y_pos, font_medium, GREEN)
        y_pos += 50

        # Button states (Face buttons)
        buttons = [(0, "□", 50), (1, "X", 170), (2, "○", 290), (3, "△", 410)]
        for btn_id, symbol, x in buttons:
            state = joystick.get_button(btn_id)
            draw_text(symbol, x, y_pos, font_large, GREEN if state else WHITE)
        y_pos += 70

        # Shoulder buttons
        shoulders = ["L1", "R1", "L2", "R2"]
        for i, name in enumerate(shoulders):
            x = 30 + i * 150
            state = joystick.get_button(4+i)
            draw_text(name, x, y_pos, font_medium, GREEN if state else WHITE)
        y_pos += 60

        # Analog sticks
        draw_text("LEFT STICK", 40, y_pos, font_small); draw_text("RIGHT STICK", 450, y_pos, font_small)
        y_pos += 30
        
        lx, ly, rx, ry = joystick.get_axis(0), joystick.get_axis(1), joystick.get_axis(2), joystick.get_axis(3)
        draw_bar(40, y_pos, lx, 160, 20, BLUE)
        draw_bar(40, y_pos+25, -ly, 160, 20, BLUE)
        draw_text(f"LX:{lx:.2f} LY:{ly:.2f}", 220, y_pos, font_small)
        
        draw_bar(450, y_pos, rx, 160, 20, YELLOW)
        draw_bar(450, y_pos+25, -ry, 160, 20, YELLOW)
        draw_text(f"RX:{rx:.2f} RY:{ry:.2f}", 630, y_pos, font_small)
        y_pos += 80

        # Triggers
        l2 = joystick.get_axis(4) if joystick.get_numaxes() > 4 else 0
        r2 = joystick.get_axis(5) if joystick.get_numaxes() > 5 else 0
        draw_text("TRIGGERS", 40, y_pos, font_medium)
        draw_bar(130, y_pos+8, l2, 200, 22, RED)
        draw_bar(130, y_pos+38, r2, 200, 22, RED)
        draw_text(f"L2:{l2:.2f}  R2:{r2:.2f}", 350, y_pos+15, font_small)
        y_pos += 80

        # 🔥 NEW: Button Press Log (Left Panel)
        draw_text("📝 RECENT PRESSES", 20, y_pos, font_medium, ORANGE)
        y_pos += 35
        draw_press_log(press_log, 20, y_pos)
        
        # D-Pad (Right Panel)
        hat = joystick.get_hat(0)
        draw_text("D-PAD", 500, y_pos, font_medium)
        dpad_x, dpad_y = hat
        if dpad_y == -1: draw_text('↑', 580, y_pos+25, font_large, GREEN)
        if dpad_y == 1:  draw_text('↓', 580, y_pos+25, font_large, GREEN)
        if dpad_x == -1: draw_text('←', 560, y_pos+45, font_large, GREEN)
        if dpad_x == 1:  draw_text('→', 600, y_pos+45, font_large, GREEN)

        # Status bar
        draw_text("TAB: Clear Log | Works unfocused ✓", 20, HEIGHT-35, font_small, GREEN)
        draw_text(f"ESP32: {ESP32_IP}:{ESP32_PORT}", WIDTH-280, HEIGHT-35, font_small, GREEN)

        pygame.display.flip()
        clock.tick(60)

except KeyboardInterrupt:
    print("\nStopped by user")

sock.close()
pygame.quit()
sys.exit()
