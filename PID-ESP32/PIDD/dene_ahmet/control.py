import os
import pygame
import socket
import time

os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

ESP32_IP = "10.224.10.188"
ESP32_PORT = 4210
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise RuntimeError("No controller detected!")

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Connected to controller: {joystick.get_name()}")

# --- UI Settings (YOUR ORIGINAL) ---
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
DEADZONE = 0.22  # FIXED for smooth forward

# Button mapping
IMPORTANT_BUTTONS = [0, 6, 11, 12, 13]
BUTTON_NAMES = {
    0: "B0", 6: "Servo Toggle", 11: "B11", 
    12: "Elev Down", 13: "Elev Up"
}

last_send_time = 0
SEND_INTERVAL = 25  # 40fps smooth

button_states = [0] * joystick.get_numbuttons()
connected = True

running = True
while running:
    pygame.event.pump()
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.JOYBUTTONDOWN or event.type == pygame.JOYBUTTONUP:
            button_states = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]

    screen.fill((20, 20, 24))
    current_time = pygame.time.get_ticks()

    # ==========================================
    # 1. VISUALIZATION (YOUR ORIGINAL UI)
    # ==========================================
    
    # Title
    title = title_font.render("ESP32 Robot Controller", True, (255, 215, 0))
    screen.blit(title, (LEFT_COLUMN_X, 10))
    subtitle = font.render(f"Connected to: {joystick.get_name()}", True, (200, 200, 200))
    screen.blit(subtitle, (LEFT_COLUMN_X, 40))
    
    # CRITICAL AXES
    title = font.render("Critical Axes (Motor Control):", True, (100, 200, 255))
    screen.blit(title, (LEFT_COLUMN_X, Y_START))
    
    axes = [
        ("0 (Turn)", joystick.get_axis(0)),
        ("2 (Back)", joystick.get_axis(2)),   
        ("4 (Spin)", joystick.get_axis(4)),     
        ("5 (Forward)", joystick.get_axis(5))   
    ]
    
    for i, (name, val) in enumerate(axes):
        if abs(val) < DEADZONE:
            val = 0
        bar_len = int((val + 1.0) * 100)
        bar_y = Y_START + 40 + i * Y_SPACING
        
        pygame.draw.rect(screen, (50, 50, 50), (LEFT_COLUMN_X, bar_y, 200, 18))
        color = (0, 180, 80) if val >= 0 else (180, 60, 0)
        pygame.draw.rect(screen, color, (LEFT_COLUMN_X, bar_y, bar_len, 18))
        text = font.render(f"Axis {name}: {val:.3f}", True, (230, 230, 230))
        screen.blit(text, (LEFT_COLUMN_X + 220, bar_y))
    
    # BUTTONS
    title = font.render("Important Buttons:", True, (100, 200, 255))
    screen.blit(title, (LEFT_COLUMN_X, Y_START + 200))
    
    for i, btn_id in enumerate(IMPORTANT_BUTTONS):
        state = button_states[btn_id]
        button_y = Y_START + 240 + i * Y_SPACING
        
        color = (220, 60, 60) if state else (80, 80, 90)
        pygame.draw.rect(screen, color, (LEFT_COLUMN_X, button_y, 20, 20), border_radius=4)
        label = BUTTON_NAMES.get(btn_id, f"Button {btn_id}")
        text = font.render(f"{label}: {'ON' if state else 'OFF'}", True, (230, 230, 230))
        screen.blit(text, (LEFT_COLUMN_X + 30, button_y))
    
    # ALL AXES DEBUG
    title = font.render("All Axes (Debug):", True, (100, 200, 255))
    screen.blit(title, (RIGHT_COLUMN_X, Y_START))
    
    for i in range(min(joystick.get_numaxes(), 12)):
        val = joystick.get_axis(i)
        bar_len = int((val + 1.0) * 100)
        bar_y = Y_START + 40 + i * Y_SPACING
        
        pygame.draw.rect(screen, (50, 50, 50), (RIGHT_COLUMN_X, bar_y, 200, 18))
        color = (0, 180, 80) if val >= 0 else (180, 60, 0)
        pygame.draw.rect(screen, color, (RIGHT_COLUMN_X, bar_y, bar_len, 18))
        text = font.render(f"Axis {i}: {val:.3f}", True, (230, 230, 230))
        screen.blit(text, (RIGHT_COLUMN_X + 220, bar_y))
    
    # ALL BUTTONS DEBUG
    title = font.render("All Buttons (Debug):", True, (100, 200, 255))
    screen.blit(title, (RIGHT_COLUMN_X, Y_START + 200))
    
    for i in range(min(joystick.get_numbuttons(), 16)):
        state = button_states[i]
        button_y = Y_START + 240 + i * Y_SPACING
        
        color = (220, 60, 60) if state else (80, 80, 90)
        pygame.draw.rect(screen, color, (RIGHT_COLUMN_X, button_y, 20, 20), border_radius=4)
        text = font.render(f"Button {i}: {'ON' if state else 'OFF'}", True, (230, 230, 230))
        screen.blit(text, (RIGHT_COLUMN_X + 30, button_y))

    # ==========================================
    # 2. NONSTOP DATA STREAM (40fps) - MODIFIED FOR R2 TRIGGER
    # ==========================================
    if current_time - last_send_time > SEND_INTERVAL:
        last_send_time = current_time
        msgs = []
        
        # R2 AXIS 5 CHECK - Send drive forward commands when R2 is pressed
        r2_value = joystick.get_axis(5)
        if abs(r2_value) > DEADZONE:
            # Send ESP32 loop() motor commands when R2 is triggered
            msgs.extend([
                "DRIVE_FORWARD",
                # Exact motor drive sequence from ESP32 loop()
                "RL_IN1 HIGH;RL_IN2 LOW;RL_PWM 255",
                "RR_IN1 LOW;RR_IN2 HIGH;RR_PWM 255", 
                "FL_IN1 HIGH;FL_IN2 LOW;FL_PWM 255",
                "FR_IN1 HIGH;FR_IN2 LOW;FR_PWM 255"
            ])
        else:
            # Send zero commands when R2 not pressed (stop motors)
            msgs.extend([
                "MOTORS_STOP",
                "RL_PWM 0",
                "RR_PWM 0",
                "FL_PWM 0", 
                "FR_PWM 0"
            ])
        
        # Send other axes/buttons as normal
        axes_to_send = [
            (0, joystick.get_axis(0)),
            (2, joystick.get_axis(2)),   
            (4, joystick.get_axis(4))    
        ]
        
        for axis_id, value in axes_to_send:
            if abs(value) < DEADZONE:
                value = 0.000
            msgs.append(f"AXIS {axis_id} {value:.3f}")

        for btn_id in IMPORTANT_BUTTONS:
            state = button_states[btn_id]
            msgs.append(f"BUTTON {btn_id} {state}")
        
        # NONSTOP STREAM
        for msg in msgs:
            sock.sendto(msg.encode(), (ESP32_IP, ESP32_PORT))

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
sock.close()
