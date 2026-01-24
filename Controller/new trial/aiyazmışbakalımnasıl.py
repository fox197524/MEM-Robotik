import pygame
import socket
import time

# --- CONFIG ---
ESP32_IP = "192.168.1.132" 
ESP32_PORT = 4210
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

running = True
while running:
    pygame.event.pump()
    for event in pygame.event.get():
        if event.type == pygame.QUIT: running = False

    # Get Axis Data
    a0 = joystick.get_axis(0)  # Rotation
    a2 = joystick.get_axis(2)  # Slide
    a4 = joystick.get_axis(4)  # Reverse Trigger
    a5 = joystick.get_axis(5)  # Forward Trigger

    # Send single CSV packet
    msg = f"{a0:.2f},{a2:.2f},{a4:.2f},{a5:.2f}"
    sock.sendto(msg.encode(), (ESP32_IP, ESP32_PORT))
    
    time.sleep(0.05) # 20Hz update is plenty
pygame.quit()