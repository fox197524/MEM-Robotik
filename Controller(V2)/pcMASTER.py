import pygame
import sys
import socket
import os
import platform

# Windows, Linux, Mac'e uygun
system = platform.system()
os.environ['SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS'] = '1'
os.environ['SDL_JOYSTICK_HIDAPI'] = '1'

if system == "Darwin":  # macOS
    os.environ['SDL_HIDAPI_DISABLED'] = '0'
elif system == "Windows":
    os.environ['SDL_VIDEODRIVER'] = 'windows'

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

# [Joystick detection - unchanged]
joystick = None
for i in range(pygame.joystick.get_count()):
    try:
        test_joy = pygame.joystick.Joystick(i)
        test_joy.init()
        name = test_joy.get_name()
        num_hats = getattr(test_joy, 'get_numhats', lambda: 0)()
        num_buttons = getattr(test_joy, 'get_numbuttons', lambda: 0)()
        num_axes = getattr(test_joy, 'get_numaxes', lambda: 0)()
        print(f"  [{i}] '{name}' | Hats:{num_hats} Btns:{num_buttons} Axes:{num_axes}")
        
        name_lower = name.lower()
        is_real_controller = (
            num_buttons >= 8 or num_hats > 0 or 
            any(keyword in name_lower for keyword in 
                ['dual', 'wireless', 'sony', 'playstation', 'controller', 'gamepad', 'xbox'])
        )
        
        if is_real_controller:
            joystick = test_joy
            print(f"[OK] SELECTED: '{name}' (ID:{i})")
            break
        test_joy.quit()
    except:
        continue

if joystick is None:
    print("Kol bulunamadı.")
    sys.exit(1)

joystick.init()
print(f"Controller '{joystick.get_name()}' READY ON {system}!")

# UI Setup
WIDTH, HEIGHT = 1100, 800
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption(f"PS5 Controller -> ESP32 ({system})")

pygame.mouse.set_visible(True)
pygame.event.set_grab(False)

# Colors
BLACK, GREEN, RED, BLUE, GRAY, WHITE, YELLOW, ORANGE = (
    (20,20,30), (0,255,100), (255,50,50), (100,150,255), 
    (100,100,120), (255,255,255), (255,255,100), (255,165,0)
)

font_small = pygame.font.SysFont(None, 20)
font_medium = pygame.font.SysFont(None, 26)
font_large = pygame.font.SysFont(None, 38)
font_press = pygame.font.SysFont(None, 18)

def draw_text(text, x, y, font, color=WHITE):
    try:
        img = font.render(str(text), True, color)
        screen.blit(img, (x, y))
        return img.get_width(), img.get_height()
    except:
        return 0, 0

def draw_bar(x, y, value, width=200, height=20, color=BLUE):
    pygame.draw.rect(screen, GRAY, (x, y, width, height), border_radius=6)
    filled = int((value + 1) * width / 2)
    pygame.draw.rect(screen, color, (x, y, max(1, filled), height), border_radius=6)

def safe_get(joy, method, *args, default=0):
    try:
        return method(*args)
    except:
        return default

def draw_press_log(presses, x, y):
    for i, (time_str, btn_name, state) in enumerate(presses[-12:]):
        color = GREEN if state == "DOWN" else RED
        pygame.draw.rect(screen, GRAY, (x, y + i*24, 450, 20), border_radius=5)
        pygame.draw.rect(screen, color, (x, y + i*24, 450, 20), 2, border_radius=5)
        draw_text(f"{time_str} | {btn_name} {state}", x+6, y + i*24 + 1, font_press, WHITE)

BUTTON_NAMES = {
    0: "SQR", 1: "CROSS", 2: "CIRC", 3: "TRI", 
    4: "L1", 5: "R1", 6: "L2", 7: "R2", 
    8: "SHARE", 9: "OPT", 10: "L3", 11: "R3", 12: "PS"
}

packets_sent = 0
press_log = []
clock = pygame.time.Clock()
running = True

print("FIXED AXIS MAPPING - PS5 Standard!")

try:
    while running:
        screen.fill(BLACK)
        current_time = pygame.time.get_ticks() / 1000
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
            elif event.type == pygame.JOYBUTTONDOWN:
                btn_id = event.button
                btn_name = BUTTON_NAMES.get(btn_id, f"BTN{btn_id}")
                press_log.append((f"{current_time:.1f}s", btn_name, "DOWN"))
                send_data(f"BUTTON {btn_id} 1")
                packets_sent += 1
            elif event.type == pygame.JOYBUTTONUP:
                btn_id = event.button
                btn_name = BUTTON_NAMES.get(btn_id, f"BTN{btn_id}")
                press_log.append((f"{current_time:.1f}s", btn_name, "UP"))
                send_data(f"BUTTON {btn_id} 0")
                packets_sent += 1
            elif event.type == pygame.JOYAXISMOTION:
                if abs(event.value) > 0.05:
                    send_data(f"AXIS {event.axis} {event.value:.2f}")
                    packets_sent += 1
            elif event.type == pygame.JOYHATMOTION:
                hat_val = safe_get(joystick, joystick.get_hat, 0)
                send_data(f"HAT 0 {hat_val}")

        num_axes = safe_get(joystick, joystick.get_numaxes, default=6)

        y_pos = 20
        draw_text("PS5 Controller -> ESP32 Dashboard", 20, y_pos, font_large, GREEN)
        y_pos += 50
        
        draw_text(f"UDP: {packets_sent} pkts | {system} | Axes: {num_axes}", 
                 20, y_pos, font_medium, YELLOW)
        y_pos += 40

        # LEFT STICK (Axis 0,1) - CORRECT
        draw_text("LEFT STICK  (Axis 0,1)", 20, y_pos, font_medium)
        lx = safe_get(joystick, joystick.get_axis, 0)  # Left X
        ly = safe_get(joystick, joystick.get_axis, 1)  # Left Y
        draw_bar(20, y_pos+30, lx, color=BLUE)
        draw_bar(20, y_pos+55, -ly, color=BLUE)  # Invert Y for display
        draw_text(f"LX0:{lx:.2f}  LY1:{ly:.2f}", 240, y_pos+35, font_small)
        y_pos += 90

        # RIGHT STICK (Axis 2,3) - CORRECT  
        draw_text("RIGHT STICK (Axis 2,3)", 20, y_pos, font_medium)
        rx = safe_get(joystick, joystick.get_axis, 2)  # Right X
        ry = safe_get(joystick, joystick.get_axis, 3)  # Right Y
        draw_bar(20, y_pos+30, rx, color=YELLOW)
        draw_bar(20, y_pos+55, -ry, color=YELLOW)
        draw_text(f"RX2:{rx:.2f}  RY3:{ry:.2f}", 240, y_pos+35, font_small)
        y_pos += 90

        # TRIGGERS (Axis 4,5) - CORRECT
        draw_text("TRIGGERS    (Axis 4,5)", 20, y_pos, font_medium)
        l2 = safe_get(joystick, joystick.get_axis, 4)  # L2
        r2 = safe_get(joystick, joystick.get_axis, 5)  # R2  
        draw_bar(20, y_pos+30, l2, color=RED)
        draw_bar(20, y_pos+55, r2, color=RED)
        draw_text(f"L2-4:{l2:.2f}  R2-5:{r2:.2f}", 240, y_pos+35, font_small)
        y_pos += 90

        # REMAINING AXES
        draw_text("EXTRA AXES", 20, y_pos, font_medium)
        y_pos += 30
        for i in range(6, num_axes):
            axis_val = safe_get(joystick, joystick.get_axis, i)
            bar_x = 20 + (i % 4) * 270
            bar_y = y_pos + ((i-6) // 4) * 35
            draw_bar(bar_x, bar_y, axis_val, color=ORANGE)
            draw_text(f"Axis{i}:{axis_val:.2f}", bar_x, bar_y+25, font_small)
        y_pos += 80

        # BIG BOTTOM PRESS LOG
        draw_text("ALL BUTTONS LOG (SQR CROSS TRI L1 R1 L2 R2 SHARE etc.)", 20, y_pos, font_medium, ORANGE)
        y_pos += 35
        draw_press_log(press_log, 20, y_pos)

        # Status
        draw_text("MOUSE FREE | ESC or CLICK to quit", 20, HEIGHT-35, font_small, GREEN)
        draw_text(f"ESP32: {ESP32_IP}:{ESP32_PORT}", WIDTH-250, HEIGHT-35, font_small, YELLOW)

        pygame.display.flip()
        clock.tick(60)

except KeyboardInterrupt:
    print("\nGoodbye!")

finally:
    sock.close()
    pygame.quit()
    sys.exit()
