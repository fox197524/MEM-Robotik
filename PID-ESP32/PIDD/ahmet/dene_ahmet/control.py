import os
import pygame
import socket

# Allow joystick events in background
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

ESP32_IP = "192.168.1.152"   # ESP32 IP address
ESP32_PORT = 4210

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise RuntimeError("No controller detected!")

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Connected to controller: {joystick.get_name()}")

# --- UI Settings ---
screen = pygame.display.set_mode((800, 700)) 
pygame.display.set_caption("PS5 Controller - ESP32 Control")
font = pygame.font.SysFont("Consolas", 16)
title_font = pygame.font.SysFont("Arial", 24, bold=True)
clock = pygame.time.Clock()

COLUMN_SPACING = 400
LEFT_COLUMN_X = 50
RIGHT_COLUMN_X = LEFT_COLUMN_X + COLUMN_SPACING
Y_START = 40
Y_SPACING = 25
DEADZONE = 0.1  # Ignore small joystick movements

# Button mapping for ESP32
IMPORTANT_BUTTONS = [0, 6, 11, 12, 13]
BUTTON_NAMES = {
    0: "B0",
    6: "Servo Toggle",
    11: "B11",
    12: "Elev Down",
    13: "Elev Up"
}

# Continuous send timer
last_send_time = 0
SEND_INTERVAL = 50  # ms

# Initialize button states
button_states = [0] * joystick.get_numbuttons()

running = True

while running:
    # Process events
    pygame.event.pump()
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.JOYBUTTONDOWN or event.type == pygame.JOYBUTTONUP:
            # Update button states
            button_states = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]

    screen.fill((20, 20, 24))
    
    # Draw title
    title = title_font.render("ESP32 Robot Controller", True, (255, 215, 0))
    screen.blit(title, (LEFT_COLUMN_X, 10))
    subtitle = font.render(f"Connected to: {joystick.get_name()}", True, (200, 200, 200))
    screen.blit(subtitle, (LEFT_COLUMN_X, 40))
    
    current_time = pygame.time.get_ticks()

    # ==========================================
    # 1. VISUALIZATION
    # ==========================================
    
    # --- CRITICAL AXES ---
    title = font.render("Critical Axes (Motor Control):", True, (100, 200, 255))
    screen.blit(title, (LEFT_COLUMN_X, Y_START))
    
    axes = [
        ("0 (Turn)", joystick.get_axis(0)),     # Left Stick X
        ("2 (Strafe)", joystick.get_axis(2)),   # Right Stick X
        ("4 (Back)", joystick.get_axis(4)),     # L2 Trigger
        ("5 (Forward)", joystick.get_axis(5))   # R2 Trigger
    ]
    
    for i, (name, val) in enumerate(axes):
        # Apply deadzone
        if abs(val) < DEADZONE:
            val = 0
            
        bar_len = int((val + 1.0) * 100)
        bar_y = Y_START + 40 + i * Y_SPACING
        
        # Background
        pygame.draw.rect(screen, (50, 50, 50), (LEFT_COLUMN_X, bar_y, 200, 18))
        # Value bar
        color = (0, 180, 80) if val >= 0 else (180, 60, 0)
        pygame.draw.rect(screen, color, (LEFT_COLUMN_X, bar_y, bar_len, 18))
        # Text
        text = font.render(f"Axis {name}: {val:.3f}", True, (230, 230, 230))
        screen.blit(text, (LEFT_COLUMN_X + 220, bar_y))
    
    # --- BUTTONS ---
    title = font.render("Important Buttons:", True, (100, 200, 255))
    screen.blit(title, (LEFT_COLUMN_X, Y_START + 200))
    
    for i, btn_id in enumerate(IMPORTANT_BUTTONS):
        state = button_states[btn_id]
        button_y = Y_START + 240 + i * Y_SPACING
        
        # Button state indicator
        color = (220, 60, 60) if state else (80, 80, 90)
        pygame.draw.rect(screen, color, (LEFT_COLUMN_X, button_y, 20, 20), border_radius=4)
        
        # Button label
        label = BUTTON_NAMES.get(btn_id, f"Button {btn_id}")
        text = font.render(f"{label}: {'ON' if state else 'OFF'}", True, (230, 230, 230))
        screen.blit(text, (LEFT_COLUMN_X + 30, button_y))
    
    # --- ALL AXES (DEBUG) ---
    title = font.render("All Axes (Debug):", True, (100, 200, 255))
    screen.blit(title, (RIGHT_COLUMN_X, Y_START))
    
    for i in range(joystick.get_numaxes()):
        val = joystick.get_axis(i)
        bar_len = int((val + 1.0) * 100)
        bar_y = Y_START + 40 + i * Y_SPACING
        
        # Background
        pygame.draw.rect(screen, (50, 50, 50), (RIGHT_COLUMN_X, bar_y, 200, 18))
        # Value bar
        color = (0, 180, 80) if val >= 0 else (180, 60, 0)
        pygame.draw.rect(screen, color, (RIGHT_COLUMN_X, bar_y, bar_len, 18))
        # Text
        text = font.render(f"Axis {i}: {val:.3f}", True, (230, 230, 230))
        screen.blit(text, (RIGHT_COLUMN_X + 220, bar_y))
    
    # --- ALL BUTTONS (DEBUG) ---
    title = font.render("All Buttons (Debug):", True, (100, 200, 255))
    screen.blit(title, (RIGHT_COLUMN_X, Y_START + 200))
    
    for i in range(joystick.get_numbuttons()):
        state = button_states[i]
        button_y = Y_START + 240 + i * Y_SPACING
        
        # Button state indicator
        color = (220, 60, 60) if state else (80, 80, 90)
        pygame.draw.rect(screen, color, (RIGHT_COLUMN_X, button_y, 20, 20), border_radius=4)
        
        # Button label
        text = font.render(f"Button {i}: {'ON' if state else 'OFF'}", True, (230, 230, 230))
        screen.blit(text, (RIGHT_COLUMN_X + 30, button_y))

    # ==========================================
    # 2. DATA SENDING
    # ==========================================
    
    if current_time - last_send_time > SEND_INTERVAL:
        last_send_time = current_time
        msgs = []
        
        # Send critical axes with deadzone applied
        axes_to_send = [
            (0, joystick.get_axis(0)),   # Turn
            (2, joystick.get_axis(2)),   # Strafe
            (4, joystick.get_axis(4)),   # Backward
            (5, joystick.get_axis(5))    # Forward
        ]
        
        for axis_id, value in axes_to_send:
            # Apply deadzone
            if abs(value) < DEADZONE:
                value = 0
            msgs.append(f"AXIS {axis_id} {value:.3f}")
        
        # Send important button states
        for btn_id in IMPORTANT_BUTTONS:
            state = button_states[btn_id]
            msgs.append(f"BUTTON {btn_id} {state}")
        
        # Send all messages
        for msg in msgs:
            sock.sendto(msg.encode(), (ESP32_IP, ESP32_PORT))
            # Uncomment for debugging:
            # print(f"Sent: {msg}")

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
