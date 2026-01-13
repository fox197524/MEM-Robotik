#!/usr/bin/env python3
"""
PS5 Controller to ESP32 WiFi Bridge
Detects PS5 controller inputs and sends them to ESP32 via WiFi
Debugging mode enabled - shows all controller inputs and WiFi communication
"""

# Import operating system functions for system commands
import os
# Import system utilities
import sys
# Import HTTP requests library for WiFi communication
import requests
# Import JSON library for data parsing
import json
# Import datetime for timestamped logging
from datetime import datetime

# ===== DEPENDENCY INSTALLATION =====
# Try to import pygame for controller detection and input handling
try:
    # Attempt to import pygame module
    import pygame
# If pygame is not installed, catch the ImportError
except ImportError:
    # Print message that pygame needs to be installed
    print("❌ pygame not installed. Installing now...")
    # Use pip to install pygame automatically
    os.system("pip install pygame")
    # Now import pygame after installation
    import pygame

# ===== ESP32 WIFI CONFIGURATION =====
# IP address of ESP32 access point (default for AP mode)
ESP32_IP = "192.168.4.1"
# HTTP port number for ESP32 web server
ESP32_PORT = 80
# Complete URL for sending keyboard input to ESP32
ESP32_URL = f"http://{ESP32_IP}:{ESP32_PORT}/keypress"

# ===== TERMINAL COLOR CODES =====
# ANSI color code for green text (success messages)
GREEN = '\033[92m'
# ANSI color code for yellow text (warnings)
YELLOW = '\033[93m'
# ANSI color code for cyan text (info)
CYAN = '\033[96m'
# ANSI color code for red text (errors)
RED = '\033[91m'
# ANSI color code to reset text color to default
RESET = '\033[0m'
# ANSI code for bold/bright text
BOLD = '\033[1m'

# ===== PS5 CONTROLLER BUTTON MAPPING =====
# Dictionary mapping button indices to human-readable button names
# Keys are button indices, values are button names for display
PS5_BUTTONS = {
    0: "CROSS",           # X button (blue square button)
    1: "CIRCLE",          # O button (red circle button)
    2: "TRIANGLE",        # Triangle button (green triangle button)
    3: "SQUARE",          # Square button (pink square button)
    4: "L1",              # L1 shoulder button (top left)
    5: "R1",              # R1 shoulder button (top right)
    6: "L2",              # L2 trigger button (left trigger)
    7: "R2",              # R2 trigger button (right trigger)
    8: "SHARE",           # Share button (left center)
    9: "OPTIONS",         # Options button (right center)
    10: "L3",             # Left stick press (L stick button)
    11: "R3",             # Right stick press (R stick button)
    12: "PS",             # PS button (center button)
    13: "TOUCHPAD",       # Touchpad button (touchpad press)
}

# ===== D-PAD MAPPING =====
# Dictionary mapping D-Pad axis values to direction names
DPAD_MAPPING = {
    -1: "UP",             # Up direction
    0: "CENTER",          # Center/neutral position
    1: "DOWN"             # Down direction
}

# ===== FUNCTION DEFINITIONS =====

def print_header():
    """
    Print application header banner to terminal
    Shows the application title and purpose
    """
    # Start new line
    print(f"\n{BOLD}{CYAN}")
    # Top border of header
    print("╔══════════════════════════════════════════╗")
    # First line of title
    print("║     PS5 Controller → ESP32 Bridge       ║")
    # Second line of title
    print("║        WiFi Communication System        ║")
    # Bottom border of header
    print("╚══════════════════════════════════════════╝")
    # Reset color and add spacing
    print(f"{RESET}\n")

def print_status(message, status="info"):
    """
    Print formatted status message with timestamp and icon
    
    Args:
        message (str): The message to display
        status (str): Type of message - 'info', 'success', 'error', 'input', or 'debug'
    """
    # Get current time in HH:MM:SS format
    timestamp = datetime.now().strftime("%H:%M:%S")
    
    # Select icon and color based on status type
    if status == "info":
        # Info icon in cyan
        icon = f"{CYAN}ℹ{RESET}"
    elif status == "success":
        # Success checkmark in green
        icon = f"{GREEN}✓{RESET}"
    elif status == "error":
        # Error X in red
        icon = f"{RED}✗{RESET}"
    elif status == "input":
        # Input arrow in yellow
        icon = f"{YELLOW}→{RESET}"
    elif status == "debug":
        # Debug symbol in cyan
        icon = f"{CYAN}[DEBUG]{RESET}"
    else:
        # Default bullet point
        icon = "•"
    
    # Print formatted message with timestamp and icon
    print(f"[{timestamp}] {icon} {message}")

def test_esp32_connection():
    """
    Test WiFi connection to ESP32 device
    
    Returns:
        bool: True if connection successful, False otherwise
    """
    # Print debug message
    print_status("Testing ESP32 connection...", "info")
    
    # Use try-except to handle potential connection errors
    try:
        # Send GET request to ESP32 status endpoint with 2 second timeout
        response = requests.get(f"http://{ESP32_IP}:{ESP32_PORT}/status", timeout=2)
        
        # Check if response code is 200 (OK)
        if response.status_code == 200:
            # Parse response as JSON
            data = response.json()
            # Print success message with SSID
            print_status(f"ESP32 Connected! SSID: {data['ssid']}", "success")
            # Print number of connected clients
            print_status(f"Connected clients: {data['connected_clients']}", "info")
            # Return success
            return True
        else:
            # Print error if response code is not 200
            print_status("ESP32 returned error response", "error")
            # Return failure
            return False
    
    # Handle connection refused error
    except requests.exceptions.ConnectionError:
        # Print error message with IP address
        print_status(f"Cannot connect to ESP32 at {ESP32_IP}:{ESP32_PORT}", "error")
        # Print troubleshooting steps
        print_status("Make sure:", "error")
        print_status("  1. ESP32 is powered on", "error")
        print_status("  2. Your Mac is connected to ESP32-S3-N16R8 WiFi", "error")
        print_status("  3. ESP32 has started correctly", "error")
        # Return failure
        return False
    
    # Handle any other exceptions
    except Exception as e:
        # Print generic error message
        print_status(f"Connection error: {e}", "error")
        # Return failure
        return False

def send_input_to_esp32(input_name, input_type="button"):
    """
    Send PS5 controller input to ESP32 via WiFi HTTP POST request
    
    Args:
        input_name (str): Name of the input (button, trigger, dpad, etc)
        input_type (str): Type of input for logging
    
    Returns:
        bool: True if sending was successful, False otherwise
    """
    # Use try-except for error handling
    try:
        # Create JSON payload with the input name
        payload = {"key": input_name.lower()}
        
        # DEBUG: Print what we're sending
        print_status(f"[DEBUG] Sending to ESP32: {json.dumps(payload)}", "debug")
        
        # Send POST request to ESP32 keypress endpoint
        response = requests.post(
            # Target URL
            ESP32_URL,
            # JSON data in request body
            json=payload,
            # Set content type header
            headers={"Content-Type": "application/json"},
            # 2 second timeout for response
            timeout=2
        )
        
        # DEBUG: Print response status code
        print_status(f"[DEBUG] ESP32 Response Code: {response.status_code}", "debug")
        
        # Check if response was successful (200 OK)
        if response.status_code == 200:
            # Parse response JSON
            data = response.json()
            # DEBUG: Print raw response
            print_status(f"[DEBUG] ESP32 Response: {json.dumps(data)}", "debug")
            # Print success message with ESP32 confirmation
            print_status(f"[{input_type.upper()}] {input_name} → {data['status']}", "success")
            # Return success
            return True
        else:
            # Print error if response code is not 200
            print_status(f"[{input_type.upper()}] {input_name} → ERROR {response.status_code}", "error")
            # Return failure
            return False
    
    # Handle timeout error
    except requests.exceptions.Timeout:
        # Print timeout error
        print_status(f"[{input_type.upper()}] {input_name} → Timeout (no response)", "error")
        # Return failure
        return False
    
    # Handle connection refused error
    except requests.exceptions.ConnectionError:
        # Print connection error
        print_status(f"[{input_type.upper()}] {input_name} → Connection failed", "error")
        # Return failure
        return False
    
    # Handle any other exceptions
    except Exception as e:
        # Print generic error with exception details
        print_status(f"[{input_type.upper()}] {input_name} → Error: {e}", "error")
        # Return failure
        return False

def detect_controllers():
    """
    Detect and initialize connected game controllers
    Scans for PS5 controller connected via USB or Bluetooth
    
    Returns:
        pygame.joystick.Joystick: Joystick object if found, None otherwise
    """
    # Print scanning message
    print_status("Scanning for PS5 controller...", "info")
    
    # Initialize pygame library for input detection
    pygame.init()
    # Initialize joystick module
    pygame.joystick.init()
    
    # Get number of connected controllers
    joystick_count = pygame.joystick.get_count()
    
    # DEBUG: Print number of detected controllers
    print_status(f"[DEBUG] Number of detected controllers: {joystick_count}", "debug")
    
    # Check if any controllers found
    if joystick_count == 0:
        # Print error message
        print_status("No controllers detected!", "error")
        # Return None
        return None
    
    # Loop through each detected controller
    for i in range(joystick_count):
        # Get joystick object at index i
        js = pygame.joystick.Joystick(i)
        # Initialize the joystick
        js.init()
        # Get controller name
        name = js.get_name()
        
        # Print controller name
        print_status(f"Found controller: {name}", "success")
        # Print number of buttons available
        print_status(f"Buttons: {js.get_numbuttons()}", "info")
        # Print number of analog axes
        print_status(f"Axes: {js.get_numaxes()}", "info")
        # Print number of hat switches (D-Pad)
        print_status(f"Hats: {js.get_numhats()}", "info")
        
        # DEBUG: Print detailed controller info
        print_status(f"[DEBUG] Controller ID: {i}", "debug")
        print_status(f"[DEBUG] Controller GUID: {js.get_guid()}", "debug")
        
        # Return the joystick object
        return js
    
    # Return None if no controller found
    return None

def run_controller_listener(joystick):
    """Main controller input listener loop"""
    print_status("Controller listener started - Press buttons on your PS5 controller", "success")
    print_status("Press CTRL+C to exit\n", "info")
    
    # Track button states to avoid duplicate sends
    button_states = {}
    axis_states = {}
    hat_states = {}
    
    try:
        while True:
            # Process pygame events
            for event in pygame.event.get():
                
                # ===== BUTTON PRESS EVENT =====
                if event.type == pygame.JOYBUTTONDOWN:
                    button_name = PS5_BUTTONS.get(event.button, f"BTN_{event.button}")
                    button_states[event.button] = True
                    print_status(f"Button pressed: {button_name}", "input")
                    send_input_to_esp32(button_name, "button")
                
                # ===== BUTTON RELEASE EVENT =====
                elif event.type == pygame.JOYBUTTONUP:
                    button_name = PS5_BUTTONS.get(event.button, f"BTN_{event.button}")
                    button_states[event.button] = False
                    print_status(f"Button released: {button_name}", "input")
                
                # ===== ANALOG STICK / TRIGGER MOTION =====
                elif event.type == pygame.JOYAXISMOTION:
                    # Get axis value (-1.0 to 1.0)
                    value = event.value
                    
                    # Only process significant movements (dead zone)
                    if abs(value) > 0.5:
                        axis_names = {
                            0: "LEFT_STICK_X",
                            1: "LEFT_STICK_Y",
                            2: "RIGHT_STICK_X",
                            3: "RIGHT_STICK_Y",
                            4: "L2_TRIGGER",
                            5: "R2_TRIGGER",
                        }
                        
                        axis_name = axis_names.get(event.axis, f"AXIS_{event.axis}")
                        
                        # Only send if state changed significantly
                        if event.axis not in axis_states or abs(axis_states[event.axis] - value) > 0.3:
                            axis_states[event.axis] = value
                            
                            # Send simplified direction
                            if "STICK" in axis_name:
                                if value < -0.5:
                                    direction = f"{axis_name}_NEG"
                                else:
                                    direction = f"{axis_name}_POS"
                                print_status(f"Analog: {direction} ({value:.2f})", "input")
                                send_input_to_esp32(direction.lower(), "analog")
                            elif "TRIGGER" in axis_name:
                                print_status(f"Trigger: {axis_name} ({value:.2f})", "input")
                                send_input_to_esp32(axis_name.lower(), "trigger")
                
                # ===== D-PAD / HAT SWITCH =====
                elif event.type == pygame.JOYHATMOTION:
                    x, y = event.value
                    
                    dpad_input = None
                    if y == 1:
                        dpad_input = "DPAD_UP"
                    elif y == -1:
                        dpad_input = "DPAD_DOWN"
                    elif x == 1:
                        dpad_input = "DPAD_RIGHT"
                    elif x == -1:
                        dpad_input = "DPAD_LEFT"
                    elif x == 0 and y == 0:
                        dpad_input = "DPAD_CENTER"
                    
                    if dpad_input:
                        print_status(f"D-Pad: {dpad_input}", "input")
                        send_input_to_esp32(dpad_input.lower(), "dpad")
            
            # Small delay to prevent CPU spinning
            pygame.time.delay(10)
    
    except KeyboardInterrupt:
        print_status("\nController listener stopped", "info")
    except Exception as e:
        print_status(f"Error in controller listener: {e}", "error")
    finally:
        pygame.quit()

def main():
    """Main application entry point"""
    print_header()
    
    # Test ESP32 connection
    if not test_esp32_connection():
        print_status("Cannot proceed without ESP32 connection", "error")
        sys.exit(1)
    
    print()
    
    # Detect controller
    joystick = detect_controllers()
    if not joystick:
        print_status("Cannot proceed without PS5 controller", "error")
        sys.exit(1)
    
    print()
    
    # Start listening to controller
    run_controller_listener(joystick)

if __name__ == "__main__":
    main()
