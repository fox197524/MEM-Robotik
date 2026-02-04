import os
import pygame
import socket
import time
import sys

os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

clock = pygame.time.Clock()
running = True

event_handlers = [
    pygame.JOYBUTTONDOWN,
    pygame.JOYBUTTONUP,
    pygame.JOYAXISMOTION,
    pygame.JOYHATMOTION
]

ESP32_IP = "10.224.10.188"
ESP32_PORT = 4210
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
def main():
    global clock
    global running
    pygame.init()
    pygame.joystick.init()
    
    if pygame.joystick.get_count() == 0:
        raise RuntimeError("No controller detected!")
    
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Connected to controller: {joystick.get_name()}")
    while running:
        events = pygame.event.get()
        for b in range(joystick.get_numbuttons()):
            if joystick.get_button(b):
                print(f"Button: {b}")
                time.sleep(1/60)
        for axis in range(joystick.get_numaxes()):
            value = joystick.get_axis(axis)
            if abs(value) > 0.1:
                print(f"Axis: {axis}: {value:.3f}")
        clock.tick(60)
    pygame.quit()    
    sys.exit()



if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("exit")
