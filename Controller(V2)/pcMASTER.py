import pygame
import sys
import socket

ESP32_IP = "172.20.10.2"   
ESP32_PORT = 4210         

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_data(message: str):
    sock.sendto(message.encode(), (ESP32_IP, ESP32_PORT))

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No controller detected.")
    sys.exit()
else:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Detected controller: {joystick.get_name()}")

WIDTH, HEIGHT = 600, 450
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("PS5 Controller Tester")
font = pygame.font.SysFont(None, 28)

def draw_text(text, x, y, color=(255,255,255)):
    img = font.render(text, True, color)
    screen.blit(img, (x, y))

def draw_bar(x, y, value):
    bar_length = 200
    filled = int((value + 1) / 2 * bar_length)
    pygame.draw.rect(screen, (100,100,100), (x, y, bar_length, 20))
    pygame.draw.rect(screen, (0,200,0), (x, y, filled, 20))

running = True
while running:
    screen.fill((30,30,30))

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    for b in range(joystick.get_numbuttons()):
        state = joystick.get_button(b)
        color = (0,200,0) if state else (200,0,0)
        draw_text(f"Button {b}: {'ON' if state else 'OFF'}", 20, 20 + b*25, color)
        send_data(f"BUTTON {b} {state}")

    for a in range(joystick.get_numaxes()):
        val = joystick.get_axis(a)
        draw_text(f"Axis {a}: {val:.2f}", 300, 20 + a*40)
        draw_bar(300, 40 + a*40, val)
        send_data(f"AXIS {a} {val:.2f}")

    for h in range(joystick.get_numhats()):
        val = joystick.get_hat(h)
        draw_text(f"D-pad {h}: {val}", 20, 300 + h*30)
        send_data(f"HAT {h} {val}")

    pygame.display.flip()
    pygame.time.Clock().tick(30)  

pygame.quit()