import os
import pygame
import socket
import time
import sys

os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

ESP32_IP = "10.224.10.188"
ESP32_PORT = 4210
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_command(fwd, strafe, turn, rot):
    message = f"{fwd:.3f},{strafe:.3f},{turn:.3f},{rot:.3f}"
    sock.sendto(message.encode(), (ESP32_IP, ESP32_PORT))
    print(f"Sent: {message}")

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise RuntimeError("No controller detected!")

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Connected to controller: {joystick.get_name()}")

clock = pygame.time.Clock()
running = True

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    fwd = -joystick.get_axis(5)
    strafe = joystick.get_axis(0)
    turn = joystick.get_axis(2)
    rot = joystick.get_axis(4)
    
    deadzone = 0.1
    if abs(fwd) < deadzone: fwd = 0
    if abs(strafe) < deadzone: strafe = 0
    if abs(turn) < deadzone: turn = 0
    if abs(rot) < deadzone: rot = 0
    
    send_command(fwd, strafe, turn, rot)
    
    if clock.get_time() > 100:
        print(f"Fwd:{fwd:.3f} Strafe:{strafe:.3f} Turn:{turn:.3f} Rot:{rot:.3f}")
    
    clock.tick(60)

pygame.quit()
sock.close()
