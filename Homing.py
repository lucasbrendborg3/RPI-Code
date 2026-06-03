#!/usr/bin/env python3
import os
# Ensure lgpio notify pipe is created in script directory
os.chdir(os.path.dirname(os.path.abspath(__file__)))

import sys
import time
import struct
import can
import RPi.GPIO as GPIO  

# --- CONFIGURATION ---
NODE_IDS      = [1, 2, 0]
HOME_PINS     = {0: 22, 1: 27, 2: 17}
SEARCH_VEL    = -5.0
SEARCH_VEL_0  =  0.3 * SEARCH_VEL
BACK_VEL      =  10.0
BACK_VEL_0    =  0.3 * BACK_VEL
CAN_IFACE     = 'can0'
BITRATE       = 250_000

VEL_GAIN      = 0.167
VEL_I_GAIN    = 0.333

# ODrive CANSimple IDs
CMD_AXIS_STATE      = 0x07  
CMD_SET_CTRL_MODE   = 0x0B  
CMD_SET_INPUT_VEL   = 0x0D  
CMD_SET_INPUT_POS   = 0x00C  
CMD_SET_ABS_POS     = 0x19  
CMD_GET_ENCODER_EST = 0x09  

AXIS_CLOSED_LOOP    = 8
CTRL_VEL            = 2
CTRL_POS            = 3
INPUT_VEL_RAMP      = 1
INPUT_PASSTHROUGH   = 0

SWITCH_OFFSETS = {
    0: -21.35,  
    1: -43.0,  
    2: -58.5   
}

# --- NEW DECELERATION CONSTANTS ---
SEARCH_CREEP_FACTOR = 0.2  # 20% speed for the second "tap"
RETURN_KP           = 2.0  # Proportional gain for slowing down
RETURN_MIN_VEL      = 0.5  # Don't slow down more than this during velocity phase
POS_CONTROL_MARGIN  = 0.1  # Distance from 0.0 to switch to strict Position Control

def open_can(iface, bitrate):
    try:
        return can.interface.Bus(interface='socketcan', channel=iface, bitrate=bitrate)
    except Exception as e:
        print("ERROR opening CAN:", e)
        sys.exit(1)

def send_cmd(bus, node, cmd_id, payload: bytes):
    arb = (node << 5) | cmd_id
    msg = can.Message(
        arbitration_id=arb, data=payload, is_extended_id=False,
        is_fd=len(payload) > 8, bitrate_switch=len(payload) > 8 
    )
    try:
        bus.send(msg)
    except can.CanOperationError as e:
        print(f"[Node {node}] ERROR sending 0x{cmd_id:02X}: {e}")

def init_axis(bus, node):
    for _ in range(3):
        send_cmd(bus, node, CMD_AXIS_STATE, struct.pack('<I', AXIS_CLOSED_LOOP))
        time.sleep(0.05)
    start = time.time()
    while time.time() - start < 2.0:
        m = bus.recv(timeout=0.5)
        if m and m.arbitration_id == ((node<<5)|0x01):
            _, st, *_ = struct.unpack('<IBBB', m.data[:7])
            if st == AXIS_CLOSED_LOOP:
                print(f"[Node {node}] ✅ Armed")
                break
    else:
        print(f"[Node {node}] ⚠️ No heartbeat, proceeding")
    send_cmd(bus, node, 0x1B, struct.pack('<ff', VEL_GAIN, VEL_I_GAIN))
    time.sleep(0.05)
    send_cmd(bus, node, CMD_SET_CTRL_MODE, struct.pack('<II', CTRL_VEL, INPUT_VEL_RAMP))
    time.sleep(0.1)

def print_telemetry(bus, node, state_msg):
    """Helper to keep terminal output clean during loops."""
    m = bus.recv(timeout=0.02)
    if m and m.arbitration_id == ((node<<5) | CMD_GET_ENCODER_EST):
        pos, vel = struct.unpack('<ff', m.data[:8])
        print(f"\r[Node {node}] {state_msg:<15} Pos: {pos:>7.3f}, Vel: {vel:>7.3f}   ", end="", flush=True)
        return pos
    return None

def home_and_back(bus, node):
    pin = HOME_PINS[node]
    offset = SWITCH_OFFSETS[node]
    sv_fast, bv_fast = (SEARCH_VEL_0, BACK_VEL_0) if node == 0 else (SEARCH_VEL, BACK_VEL)
    sv_creep = sv_fast * SEARCH_CREEP_FACTOR
    
    # --- 1. FAST SEARCH (First Tap) ---
    send_cmd(bus, node, CMD_SET_INPUT_VEL, struct.pack('<f', sv_fast))
    while GPIO.input(pin) == GPIO.LOW:
        print_telemetry(bus, node, "Fast Search")
    print()

    # --- 2. STOP & BACK OFF ---
    send_cmd(bus, node, CMD_SET_INPUT_VEL, struct.pack('<f', 0.0))
    time.sleep(0.1)
    
    send_cmd(bus, node, CMD_SET_INPUT_VEL, struct.pack('<f', -sv_fast)) # Reverse direction
    while GPIO.input(pin) == GPIO.HIGH: # Wait for switch to untrip
        print_telemetry(bus, node, "Backing Off")
    time.sleep(0.2) # Clear it by just a fraction more
    send_cmd(bus, node, CMD_SET_INPUT_VEL, struct.pack('<f', 0.0))
    time.sleep(0.1)
    print()

    # --- 3. CREEP SEARCH (Second Tap) ---
    send_cmd(bus, node, CMD_SET_INPUT_VEL, struct.pack('<f', sv_creep))
    while GPIO.input(pin) == GPIO.LOW:
        print_telemetry(bus, node, "Creep Search")
    
    send_cmd(bus, node, CMD_SET_INPUT_VEL, struct.pack('<f', 0.0))
    time.sleep(0.1)
    print()

    # --- 4. CALIBRATE ---
    send_cmd(bus, node, CMD_SET_ABS_POS, struct.pack('<f', offset))
    time.sleep(0.1)

    # --- 5. SMOOTH RETURN (P-Control) ---
    current_pos = offset
    while True:
        pos = print_telemetry(bus, node, "Returning")
        if pos is not None:
            current_pos = pos
            dist_to_home = 0.0 - current_pos
            
            # THE LANDING GEAR: If we are extremely close, break the loop
            if dist_to_home < POS_CONTROL_MARGIN:
                break
                
            dynamic_bv = max(RETURN_MIN_VEL, min(bv_fast, dist_to_home * RETURN_KP))
            send_cmd(bus, node, CMD_SET_INPUT_VEL, struct.pack('<f', dynamic_bv))
            
    print()

    # --- 6. POSITION CONTROL LOCK (The final stretch) ---
    print(f"[Node {node}] Handing off to Position Controller...")
    send_cmd(bus, node, CMD_SET_CTRL_MODE, struct.pack('<II', CTRL_POS, INPUT_PASSTHROUGH))
    time.sleep(0.05)
    send_cmd(bus, node, CMD_SET_INPUT_POS, struct.pack('<fhh', 0.0, 0, 0)) 
    time.sleep(0.5) # Give it half a second to perfectly settle
    
    print(f"[Node {node}] ✅ Reached and Locked at Home (0.0)")

def read_encoder(bus, node):
    start = time.time()
    while time.time() - start < 0.2:
        m = bus.recv(timeout=0.05)
        if m and m.arbitration_id == ((node<<5)|CMD_GET_ENCODER_EST):
            pos, vel = struct.unpack('<ff', m.data)
            return pos, vel
    return None, None

if __name__ == '__main__':
    GPIO.setmode(GPIO.BCM)
    for p in HOME_PINS.values():
        GPIO.setup(p, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    bus = open_can(CAN_IFACE, BITRATE)
    while bus.recv(timeout=0) is not None:
        pass

    for n in NODE_IDS:
        init_axis(bus, n)
        home_and_back(bus, n)

    print("\nEncoder estimates:")
    for n in NODE_IDS:
        pos, vel = read_encoder(bus, n)
        if pos is None:
            print(f"  Node {n}: no estimate received")
        else:
            print(f"  Node {n}: pos={pos:.4f} turns, vel={vel:.4f} turns/s")

    GPIO.cleanup()