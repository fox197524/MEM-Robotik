
#!/usr/bin/env python3
import pygame
import sys
import socket
import os
import platform
import time

# 🔥 CROSS-PLATFORM JOYSTICK FIXES
system = platform.system()
print(f"🖥️  Starting on {system}...")

# Universal SDL fixes for ALL platforms
os.environ['SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS'] = '1'
os.environ['SDL_JOYSTICK_HIDAPI'] = '1'

if system == "Darwin":  # macOS
    os.environ['SDL_HIDAPI_DISABLED'] = '0'
    print("🍎 macOS mode")
elif system == "Windows":
    os.environ['SDL_VIDEODRIVER'] = 'windows'
    print("🪟 Windows mode")
else:  # Linux/Wayland
    print("🐧 Linux mode")

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

print(f"Scanning controllers... Found {pygame.joystick.get_count()} devices")

# 🔥 UNIVERSAL JOYSTICK DETECTOR
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
        
        # Cross-platform controller detection
        name_lower = name.lower()
        is_real_controller = (
            num_buttons >= 8 or num_hats > 0 or 
            any(keyword in name_lower for keyword in 
                ['dual', 'wireless', 'sony', 'playstation', 'controller', 'gamepad'])
        )
        
        if is_real_controller:
            joystick = test_joy
            print(f"✅ SELECTED: '{name}' (ID:{i})")
            break
            
        test_joy.quit()
    except Exception as e:
        print(f"  [{i}] Failed: {e}")
        continue

if joystick is None:
    print("❌ No gamepad found!")
    print("\n💡 TROUBLESHOOTING:")
    if system == "Darwin":
        print("  • Connect via USB (most reliable)")
        print("  • System Settings → Privacy → Input Monitoring → Allow Terminal/Python")
    elif system == "Windows":
        print("  • Check Device Manager → Human Interface Devices")
        print("  • Install DS4Windows or Steam controller config")
    else:
        print("  • Ensure controller paired via Bluetooth")
        print("  • Try: SDL_VIDEODRIVER=x11 python3 script.py")
    sys.exit(1)

joystick.init()
print(f"🎮 '{joystick.get_name()}' READY ON {system}!")

# Universal UI
WIDTH, HEIGHT = 1000, 750
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption(f"PS5 Controller → ESP32 ({system})")

# Enable background input (all platforms)
try:
    pygame.event.set_grab(True)
except:
    pass

# Colors
BLACK, GREEN, RED, BLUE, GRAY, WHITE, YELLOW, ORANGE = (
    (20,20,30), (0,255,100), (255,50,50), (100,150,255), 
    (100,100,120), (255,255,255), (255,255,100), (255,165,0)
)

font_small = pygame.font.SysFont(None, 22)
font_medium = pygame.font.SysFont(None, 28)
font_large = pygame.font.SysFont(None, 42)
font_press = pygame.font.SysFont(None, 20)

def draw_text(text, x, y, font, color=WHITE):
    try:
        img = font.render(str(text), True, color)
        screen.blit(img, (x, y))
        return img.get_width(), img.get_height()
    except:
        return 0, 0

def safe_get(joy, method, *args, default=None):
    """Safe cross-platform access"""
    try:
        return method(*args)
    except:
        return default

def draw_bar(x, y, value, width=180, height=22, color=GREEN):
    pygame.draw.rect(screen, GRAY, (x, y, width, height), border_radius=8)
    filled = int((value + 1) * width / 2)
    pygame.draw.rect(screen, color, (x, y, max(1, filled), height), border_radius=8)

def draw_press_log(presses, x, y):
    for i, (time_str, btn_name, state) in enumerate(presses[-10:]):
        color = GREEN if state == "DOWN" else RED
        pygame.draw.rect(screen, GRAY, (x, y + i*28, 320, 24), border_radius=6)
        pygame.draw.rect(screen, color, (x, y + i*28, 320, 24), 3, border_radius=6)
        draw_text(f"{time_str} | {btn_name}", x+8, y + i*28 + 3, font_press, WHITE)

# Universal button names
BUTTON_NAMES = {
    0: "□", 1: "X", 2: "○", 3: "△", 4: "L1", 5: "R1", 
    6: "L2", 7: "R2", 8: "Share", 9: "Options", 10: "L3", 11: "R3", 12: "PS"
}

packets_sent = 0
press_log = []
clock = pygame.time.Clock()
running = True

print("🚀 CROSS-PLATFORM DASHBOARD ACTIVE!")

try:
    while running:
        screen.fill(BLACK)
        current_time = pygame.time.get_ticks() / 1000
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.JOYBUTTONDOWN:
                btn_id = event.button
                btn_name = BUTTON_NAMES.get(btn_id, f"Btn{btn_id}")
                press_log.append((f"{current_time:.1f}s", btn_name, "DOWN"))
                send_data(f"BUTTON {btn_id} 1")
                packets_sent += 1
                print(f"[{system}] BTN {btn_id} ({btn_name}) ↓")
            elif event.type == pygame.JOYBUTTONUP:
                btn_id = event.button
                btn_name = BUTTON_NAMES.get(btn_id, f"Btn{btn_id}")
                press_log.append((f"{current_time:.1f}s", btn_name, "UP"))
                send_data(f"BUTTON {btn_id} 0")
                packets_sent += 1
            elif event.type == pygame.JOYAXISMOTION:
                if abs(event.value) > 0.05:
                    send_data(f"AXIS {event.axis} {event.value:.2f}")
                    packets_sent += 1
            elif event.type == pygame.JOYHATMOTION:
                hat_val = safe_get(joystick, joystick.get_hat, 0, default=(0,0))
                send_data(f"HAT 0 {hat_val}")
                press_log.append((f"{current_time:.1f}s", f"D-Pad{hat_val}", "HAT"))

        # UI Layout
        y_pos = 20
        draw_text("🎮 Universal PS5 Controller → ESP32", 20, y_pos, font_large, GREEN)
        y_pos += 55
        
        draw_text(f"📡 UDP: {packets_sent} pkts | {system} | '{joystick.get_name()}'", 
                 20, y_pos, font_medium, YELLOW)
        y_pos += 45
        
        draw_text("✅ Background input enabled", 20, y_pos, font_medium, GREEN)
        y_pos += 50

        # Face buttons
        for btn_id, symbol, x in [(0,"□",60), (1,"X",180), (2,"○",300), (3,"△",420)]:
            state = safe_get(joystick, joystick.get_button, btn_id, default=False)
            draw_text(symbol, x, y_pos, font_large, GREEN if state else WHITE)
        y_pos += 80

        # Sticks
        lx = safe_get(joystick, joystick.get_axis, 0, default=0)
        ly = safe_get(joystick, joystick.get_axis, 1, default=0)
        rx = safe_get(joystick, joystick.get_axis, 2, default=0)
        draw_text("Sticks", 60, y_pos, font_medium)
        draw_bar(160, y_pos+5, lx)
        draw_text(f"LX:{lx:.2f} LY:{ly:.2f}", 370, y_pos, font_small)
        draw_bar(500, y_pos+5, rx)
        y_pos += 80

        # Press log
        draw_text("📝 RECENT PRESSES (Last 10)", 20, y_pos, font_medium, ORANGE)
        y_pos += 35
        draw_press_log(press_log, 20, y_pos)

        # Status bar
        draw_text(f"ESP32: {ESP32_IP}:{ESP32_PORT} ✓", 20, HEIGHT-35, font_small, GREEN)
        draw_text("Press Ctrl+C to quit", WIDTH-200, HEIGHT-35, font_small, WHITE)

        pygame.display.flip()
        clock.tick(60)

except KeyboardInterrupt:
    print("\n👋 Goodbye!")

finally:
    sock.close()
    pygame.quit()
    sys.exit()
