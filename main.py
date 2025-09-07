# Mini-SwerveDrive-1895 -- ESP32 MicroPython
# - Hosts Wi-Fi AP + web joystick UI
# - Uses L298N-style drivers (2 direction pins + 1 EN PWM per motor)
# - SSD1306 0.91" OLED (assumed 128x32 @ 0x3C) optional
# - Attempts to detect a common 9-DOF/motion sensor (MPU6050) on I2C (optional)
#
# IMPORTANT:
# - Edit the PIN MAP section below to match your wiring before connecting motors.
# - Use ADC1 pins (GPIO32..GPIO39) for analog sensors when Wi-Fi is active (ADC2 is shared with Wi-Fi).
# - Test with motors disconnected to verify the web UI and sensor readings first.
# - L298N EN pins expect PWM (3.3V logic), motor supply must be separate and common-grounded.

import network
import socket
import _thread
import time
import ujson
import machine
import hashlib
import binascii
import struct
from machine import Pin, ADC, PWM, I2C

# ---------- PIN MAP (EDIT BEFORE USE) ----------
FL_HeadingSensor_A_pin = 32
FL_HeadingSensor_B_pin = 33
FR_HeadingSensor_A_pin = 34
FR_HeadingSensor_B_pin = 35
BL_HeadingSensor_A_pin = 36
BL_HeadingSensor_B_pin = 39
BR_HeadingSensor_A_pin = 37  # set to None if pin not available on your board
BR_HeadingSensor_B_pin = 38  # set to None if pin not available on your board

# I2C pins for OLED / IMU (default common: SDA=21, SCL=22)
I2C_SDA_PIN = 21
I2C_SCL_PIN = 22
OLED_WIDTH = 128
OLED_HEIGHT = 32
OLED_I2C_ADDR = 0x3C

# ------------------ MOTOR PIN ASSIGNMENTS ------------------
FL_drive = (16, 17, 25)
FR_drive = (18, 19, 14)
BL_drive = (26, 27, 12)
BR_drive = (13, 15, 2)

FL_steer = (4, 5, 32)
FR_steer = (33, 34, 14)
BL_steer = (35, 36, 27)
BR_steer = (39, 37, 13)

# Wi-Fi AP credentials
AP_SSID = "Mini-RC-SwerveDrive-AP"
AP_PASSWORD = "FRC_TEAM1895"
AP_CHANNEL = 6

# ----------- Static IP configuration for AP -------------
AP_STATIC_IP = '192.168.10.1'
AP_NETMASK   = '255.255.255.0'
AP_GATEWAY   = '192.168.10.1'
AP_DNS       = '8.8.8.8'

# Global control state updated by web UI
control_state = {
    "left": {"x": 0.0, "y": 0.0},
    "right": {"x": 0.0, "y": 0.0},
    "buttons": {"home": False, "rotate": False, "demo": False, "leftJoystick": False, "rightJoystick": False}
}
control_lock = _thread.allocate_lock()

# WebSocket globals
websocket_clients = []
network_polling_delay = 30 # ms between polling events

ws_clients_lock = _thread.allocate_lock()

# WebSocket magic string for handshake
WS_MAGIC_STRING = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)

def map_range(val, a1, a2, b1, b2):
    return b1 + (float(val - a1) / (a2 - a1)) * (b2 - b1)

def make_adc(pin_no):
    if pin_no is None:
        return None
    try:
        adc = ADC(Pin(pin_no))
        adc.atten(ADC.ATTN_11DB)
        adc.width(ADC.WIDTH_12BIT)
        return adc
    except Exception as e:
        print("ADC init failed for pin", pin_no, ":", e)
        return None

adc_FL_A = make_adc(FL_HeadingSensor_A_pin) if FL_HeadingSensor_A_pin is not None else None
adc_FL_B = make_adc(FL_HeadingSensor_B_pin) if FL_HeadingSensor_B_pin is not None else None
adc_FR_A = make_adc(FR_HeadingSensor_A_pin) if FR_HeadingSensor_A_pin is not None else None
adc_FR_B = make_adc(FR_HeadingSensor_B_pin) if FR_HeadingSensor_B_pin is not None else None
adc_BL_A = make_adc(BL_HeadingSensor_A_pin) if BL_HeadingSensor_A_pin is not None else None
adc_BL_B = make_adc(BL_HeadingSensor_B_pin) if BL_HeadingSensor_B_pin is not None else None
adc_BR_A = make_adc(BR_HeadingSensor_A_pin) if BR_HeadingSensor_A_pin is not None else None
adc_BR_B = make_adc(BR_HeadingSensor_B_pin) if BR_HeadingSensor_B_pin is not None else None

def read_analog_as_1023(adc):
    if adc is None:
        return 0
    raw = adc.read()
    return int(raw * 1023 // 4095)

pwm_freq = 2000

def make_pwm(pin_no):
    if pin_no is None:
        return None
    try:
        p = PWM(Pin(pin_no), freq=pwm_freq, duty=0)
    except TypeError:
        pwm_pin = Pin(pin_no, Pin.OUT)
        p = PWM(pwm_pin)
        p.freq(pwm_freq)
        p.duty(0)
    return p

def pwm_set_duty(pwm_obj, duty_val):
    if pwm_obj is None:
        return
    try:
        pwm_obj.duty(int(duty_val))
    except Exception:
        try:
            pwm_obj.duty_u16(int(duty_val * 65535 // 1023))
        except Exception:
            pass

def set_pin(pin_no, val):
    if pin_no is None:
        return
    try:
        Pin(pin_no, Pin.OUT).value(1 if val else 0)
    except Exception:
        pass

def set_drive_dir(forward, dirA_pin, dirB_pin):
    set_pin(dirA_pin, forward)
    set_pin(dirB_pin, not forward)

def set_steer_dir(cw, dirA_pin, dirB_pin):
    set_pin(dirA_pin, 1 if cw else 0)
    set_pin(dirB_pin, 0 if cw else 1)

def init_motor_struct(entry):
    dirA, dirB, pwm_pin = entry
    return {
        "dirA": dirA,
        "dirB": dirB,
        "pwm_pin": pwm_pin,
        "pwm": make_pwm(pwm_pin) if pwm_pin is not None else None
    }

drive_FL = init_motor_struct(FL_drive)
drive_FR = init_motor_struct(FR_drive)
drive_BL = init_motor_struct(BL_drive)
drive_BR = init_motor_struct(BR_drive)

steer_FL = init_motor_struct(FL_steer)
steer_FR = init_motor_struct(FR_steer)
steer_BL = init_motor_struct(BL_steer)
steer_BR = init_motor_struct(BR_steer)

def set_wheel_speed_struct(drive_struct, local_motor_speed):
    pwm_obj = drive_struct["pwm"]
    dirA = drive_struct["dirA"]
    dirB = drive_struct["dirB"]
    if pwm_obj is None:
        return
    speed = int(local_motor_speed)
    if speed >= 0:
        set_drive_dir(True, dirA, dirB)
        duty = min(1023, speed * 4)
    else:
        set_drive_dir(False, dirA, dirB)
        duty = min(1023, (-speed) * 4)
    pwm_set_duty(pwm_obj, duty)

def run_steer_motor_struct(steer_struct, local_motor_speed):
    pwm_obj = steer_struct["pwm"]
    dirA = steer_struct["dirA"]
    dirB = steer_struct["dirB"]
    if pwm_obj is None:
        return
    speed = int(local_motor_speed)
    if speed >= 0:
        set_steer_dir(False, dirA, dirB)
        duty = min(1023, speed * 4)
    else:
        set_steer_dir(True, dirA, dirB)
        duty = min(1023, (-speed) * 4)
    pwm_set_duty(pwm_obj, duty)

def stop_all_drive():
    for s in (drive_FL, drive_FR, drive_BL, drive_BR):
        if s["pwm"] is not None:
            pwm_set_duty(s["pwm"], 0)

def stop_all_steer():
    for s in (steer_FL, steer_FR, steer_BL, steer_BR):
        if s["pwm"] is not None:
            pwm_set_duty(s["pwm"], 0)

def calculateHeading(sensorValueA0, sensorValueA1):
    lowThreshold = 200
    highThreshold = 760
    lineSlope = 0.342205323
    lineIntercept = 169
    if (sensorValueA0 > lowThreshold) and (sensorValueA0 < highThreshold):
        sensorValueA0Component = lineSlope * sensorValueA0 - lineIntercept
        sensorValueA1Component = 0
    else:
        sensorValueA0Component = 0
        sensorValueA1Component = -lineSlope * sensorValueA1 + 351.1
        if sensorValueA1Component > 180:
            sensorValueA1Component = sensorValueA1Component - 360
    heading = sensorValueA0Component + sensorValueA1Component
    return heading

def add_wheel_offset_by_adc(adc_obj, raw_angle):
    if adc_obj is adc_FL_A:
        offset_angle = 180
        corrected = raw_angle + offset_angle
        if corrected >= 180: corrected -= 360
        return corrected
    if adc_obj is adc_FR_A:
        offset_angle = -90
        corrected = raw_angle + offset_angle
        if corrected <= -180: corrected += 360
        return corrected
    if adc_obj is adc_BL_A:
        offset_angle = 90
        corrected = raw_angle + offset_angle
        if corrected >= 180: corrected -= 360
        return corrected
    if adc_obj is adc_BR_A:
        return raw_angle
    return raw_angle

def readCurrentHeading(adcA_obj, adcB_obj):
    if (adcA_obj is None) or (adcB_obj is None):
        return 0.0
    a = read_analog_as_1023(adcA_obj)
    b = read_analog_as_1023(adcB_obj)
    raw = calculateHeading(a, b)
    corrected = add_wheel_offset_by_adc(adcA_obj, raw)
    return corrected

def web_axis_to_analog1023(x):
    return int((x + 1.0) * 511.5)

def get_right_joystick_x_control_value():
    control_lock.acquire()
    try:
        v = control_state['right']['x']
    finally:
        control_lock.release()
    return web_axis_to_analog1023(v)

def get_right_joystick_y_control_value():
    control_lock.acquire()
    try:
        v = control_state['right']['y']
    finally:
        control_lock.release()
    return web_axis_to_analog1023(v)

def get_left_joystick_x_control_value():
    control_lock.acquire()
    try:
        v = control_state['left']['x']
    finally:
        control_lock.release()
    return web_axis_to_analog1023(v)

def convert_right_joystick_to_heading_value(right_joystick_x_value):
    joystick_x_middle_value = 510
    joystick_deadzone = 5
    if right_joystick_x_value > (joystick_x_middle_value + joystick_deadzone):
        return (right_joystick_x_value - joystick_x_middle_value) * (90.0 / (1023 - joystick_x_middle_value))
    elif right_joystick_x_value < (joystick_x_middle_value - joystick_deadzone):
        return (right_joystick_x_value - joystick_x_middle_value) * (-90.0 / joystick_x_middle_value)
    else:
        return 0.0

def set_wheel_heading_generic(steer_struct, desired_heading, current_heading):
    heading_alignment_tolerance = 20
    base_speed = 130
    heading_difference = desired_heading - current_heading
    if heading_difference > 180:
        heading_difference -= 360
    if heading_difference < -180:
        heading_difference += 360
    if abs(heading_difference) <= heading_alignment_tolerance:
        steer_speed = 0
    else:
        steer_speed = int(base_speed if heading_difference < 0 else -base_speed)
    run_steer_motor_struct(steer_struct, steer_speed)

def driveMotorToHome(timeout_ms=10000):
    start = time.ticks_ms()
    while True:
        FL_h = readCurrentHeading(adc_FL_A, adc_FL_B)
        FR_h = readCurrentHeading(adc_FR_A, adc_FR_B)
        BL_h = readCurrentHeading(adc_BL_A, adc_BL_B)
        BR_h = readCurrentHeading(adc_BR_A, adc_BR_B)

        done = True
        for adcA, adcB, steer_struct, h in (
            (adc_FL_A, adc_FL_B, steer_FL, FL_h),
            (adc_FR_A, adc_FR_B, steer_FR, FR_h),
            (adc_BL_A, adc_BL_B, steer_BL, BL_h),
            (adc_BR_A, adc_BR_B, steer_BR, BR_h),
        ):
            if adcA is None or adcB is None:
                continue
            if abs(h) <= 5:
                run_steer_motor_struct(steer_struct, 0)
            else:
                done = False
                run_steer_motor_struct(steer_struct, 120 if h < 0 else -120)
        if done:
            stop_all_steer()
            return True
        if time.ticks_diff(time.ticks_ms(), start) > timeout_ms:
            stop_all_steer()
            return False
        time.sleep_ms(50)

i2c = I2C(scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN))
oled = None
try:
    import ssd1306
    oled = ssd1306.SSD1306_I2C(OLED_WIDTH, OLED_HEIGHT, i2c, addr=OLED_I2C_ADDR)
    oled.fill(0)
    oled.text("Mini SwerveDrive", 0, 0)
    oled.show()
except Exception as e:
    oled = None

imu_present = False
imu_addr = None
try:
    devices = i2c.scan()
    if 0x68 in devices:
        imu_present = True
        imu_addr = 0x68
        try:
            who = i2c.readfrom_mem(imu_addr, 0x75, 1)
            if who and who[0] == 0x68:
                print("MPU6050 detected at 0x68")
        except Exception:
            pass
    else:
        pass
except Exception as e:
    print("I2C scan failed:", e)

def load_html_page():
    paths = ('web/index.html', '/web/index.html')
    html = None
    for p in paths:
        try:
            with open(p, 'r') as f:
                html = f.read()
                break
        except Exception:
            pass
    if html is None:
        body = "<html><body><h3>Mini SwerveDrive</h3><p>UI file missing.</p></body></html>"
        return "HTTP/1.0 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n\r\n" + body
    # Inject runtime values
    html = html.replace('__SSID__', AP_SSID)
    html = html.replace('__PASSWORD__', AP_PASSWORD)
    return "HTTP/1.0 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n\r\n" + html

def start_access_point():
    ap = network.WLAN(network.AP_IF)
    ap.active(True)
    ap.config(essid=AP_SSID, authmode=network.AUTH_WPA_WPA2_PSK, password=AP_PASSWORD, channel=AP_CHANNEL)
    # ---- Set static IP address ----
    ap.ifconfig((AP_STATIC_IP, AP_NETMASK, AP_GATEWAY, AP_DNS))
    for _ in range(50):
        if ap.active():
            break
        time.sleep_ms(100)
    print("AP config:", ap.ifconfig())
    if oled:
        try:
            oled.fill(0)
            oled.text("AP started", 0, 0)
            oled.text(AP_SSID, 0, 10)
            oled.text(ap.ifconfig()[0], 0, 20)
            oled.show()
        except Exception:
            pass
    return ap

def websocket_hash(key):
    """Generate WebSocket accept key"""
    combined = key + WS_MAGIC_STRING
    sha1 = hashlib.sha1(combined.encode()).digest()
    return binascii.b2a_base64(sha1).decode().strip()

def send_websocket_frame(client, data):
    """Send WebSocket frame to client"""
    try:
        if isinstance(data, str):
            data = data.encode('utf-8')
        
        length = len(data)
        if length < 126:
            header = struct.pack('!BB', 0x81, length)
        elif length < 65536:
            header = struct.pack('!BBH', 0x81, 126, length)
        else:
            header = struct.pack('!BBQ', 0x81, 127, length)
        
        client.send(header + data)
        return True
    except Exception as e:
        print("WebSocket send error:", e)
        return False

def parse_websocket_frame(data):
    """Parse incoming WebSocket frame"""
    if len(data) < 2:
        return None
    
    byte1, byte2 = struct.unpack('!BB', data[:2])
    opcode = byte1 & 0x0F
    masked = bool(byte2 & 0x80)
    payload_length = byte2 & 0x7F
    
    header_length = 2
    if payload_length == 126:
        if len(data) < 4:
            return None
        payload_length = struct.unpack('!H', data[2:4])[0]
        header_length = 4
    elif payload_length == 127:
        if len(data) < 10:
            return None
        payload_length = struct.unpack('!Q', data[2:10])[0]
        header_length = 10
    
    if masked:
        if len(data) < header_length + 4:
            return None
        mask = data[header_length:header_length+4]
        header_length += 4
    
    if len(data) < header_length + payload_length:
        return None
    
    payload = data[header_length:header_length+payload_length]
    
    if masked:
        payload = bytes(payload[i] ^ mask[i % 4] for i in range(len(payload)))
    
    result = payload.decode('utf-8') if opcode == 1 else payload

    return result

def handle_websocket_client(client):
    """Handle WebSocket client in separate thread"""
    try:
        while True:
            try:
                data = client.recv(1024)
                if not data:
                    break
                
                message = parse_websocket_frame(data)
                if message:
                    try:
                        control_data = ujson.loads(message)
                        control_lock.acquire()
                        try:
                            if 'left' in control_data:
                                control_state['left']['x'] = float(control_data['left'].get('x', 0.0))
                                control_state['left']['y'] = float(control_data['left'].get('y', 0.0))
                            if 'right' in control_data:
                                control_state['right']['x'] = float(control_data['right'].get('x', 0.0))
                                control_state['right']['y'] = float(control_data['right'].get('y', 0.0))
                            if 'buttons' in control_data:
                                for b in ('home', 'rotate', 'demo', 'leftJoystick', 'rightJoystick'):
                                    control_state['buttons'][b] = bool(control_data['buttons'].get(b, False))
                        finally:
                            control_lock.release()
                    except Exception as e:
                        print("JSON parse error:", e)
            except Exception as e:
                print("WebSocket receive error:", e)
                break
            
            time.sleep_ms(websocket_polling_delay)
    except Exception as e:
        print("WebSocket client error:", e)
    finally:
        # Remove client from list
        ws_clients_lock.acquire()
        try:
            if client in websocket_clients:
                websocket_clients.remove(client)
        finally:
            ws_clients_lock.release()
        try:
            client.close()
        except:
            pass
        print("WebSocket client disconnected")

def http_server_thread():
    addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(addr)
    s.listen(1)
    print("HTTP/WebSocket server listening on", addr)
    
    while True:
        try:
            cl, addr = s.accept()
            cl_file = cl.makefile('rwb', 0)
            request_line = cl_file.readline()
            if not request_line:
                cl.close()
                continue
            
            try:
                request = request_line.decode()
                method, path, _ = request.split()[:3]
            except Exception:
                cl.close()
                continue
            
            # Parse headers
            headers = {}
            while True:
                header = cl_file.readline()
                if not header or header == b'\r\n':
                    break
                try:
                    h = header.decode().strip()
                    if ':' in h:
                        key, value = h.split(':', 1)
                        headers[key.strip().lower()] = value.strip()
                except Exception:
                    pass
            
            # Handle WebSocket upgrade
            if (headers.get('upgrade', '').lower() == 'websocket' and 
                headers.get('connection', '').lower() == 'upgrade'):
                
                ws_key = headers.get('sec-websocket-key')
                if ws_key:
                    accept_key = websocket_hash(ws_key)
                    response = (
                        "HTTP/1.1 101 Switching Protocols\r\n"
                        "Upgrade: websocket\r\n"
                        "Connection: Upgrade\r\n"
                        f"Sec-WebSocket-Accept: {accept_key}\r\n"
                        "\r\n"
                    )
                    cl.send(response.encode())
                    
                    # Add client to WebSocket clients list
                    ws_clients_lock.acquire()
                    try:
                        websocket_clients.append(cl)
                    finally:
                        ws_clients_lock.release()
                    
                    print("WebSocket client connected")
                    # Handle WebSocket client in new thread
                    _thread.start_new_thread(handle_websocket_client, (cl,))
                    continue
                else:
                    cl.send("HTTP/1.0 400 Bad Request\r\n\r\n".encode())
            
            # Handle regular HTTP requests
            elif method == 'GET':
                cl.send(load_html_page().encode('utf-8'))
                cl.close()
            else:
                cl.send("HTTP/1.0 404 Not Found\r\n\r\n".encode())
                cl.close()
                
        except Exception as e:
            print("Server error:", e)
            try:
                cl.close()
            except:
                pass
            time.sleep_ms(websocket_polling_delay)

normal_drive_mode = True
rotate_drive_mode = False
demonstration_mode = False

def send_status_to_clients(status_data):
    """Send status update to all connected WebSocket clients"""
    ws_clients_lock.acquire()
    try:
        clients_to_remove = []
        for client in websocket_clients[:]:  # Create a copy to iterate over
            try:
                message = ujson.dumps(status_data)
                if not send_websocket_frame(client, message):
                    clients_to_remove.append(client)
            except Exception as e:
                print(f"Error sending to client: {e}")
                clients_to_remove.append(client)
        
        # Remove failed clients
        for client in clients_to_remove:
            if client in websocket_clients:
                websocket_clients.remove(client)
            try:
                client.close()
            except:
                pass
    finally:
        ws_clients_lock.release()

def enable_periodic_oled_update(interval_ms=500):
    if not oled:
        return
    now = time.ticks_ms()
    if not hasattr(enable_periodic_oled_update, "next"):
        enable_periodic_oled_update.next = now
    if time.ticks_diff(now, enable_periodic_oled_update.next) >= 0:
        enable_periodic_oled_update.next = time.ticks_add(now, interval_ms)
        try:
            oled.fill(0)
            oled.text("Mini SwerveDrive", 0, 0)
            oled.text("Mode:" + ("ROT" if rotate_drive_mode else ("DEMO" if demonstration_mode else "NORM")), 0, 10)
            rx = control_state['right']['x']
            ry = control_state['right']['y']
            oled.text("R:{:.2f},{:.2f}".format(rx, ry), 0, 20)
            oled.show()
        except Exception:
            pass
        
        # Send status update to WebSocket clients
        try:
            status_data = {
                "type": "status",
                "mode": "ROT" if rotate_drive_mode else ("DEMO" if demonstration_mode else "NORM"),
                "joystick": {"right_x": rx, "right_y": ry},
                "connected_clients": len(websocket_clients)
            }
            send_status_to_clients(status_data)
        except Exception:
            pass

def main_loop():
    global normal_drive_mode, rotate_drive_mode, demonstration_mode
    joystick_center = 510
    joystick_deadzone = 5
    last_log_time = time.ticks_ms()
    log_interval = 2000  # 2000ms = 2 seconds
    
    # Initialize button state tracking
    home_button_last_state = False
    rotate_button_last_state = False
    demo_button_last_state = False
    left_joystick_button_last_state = False
    right_joystick_button_last_state = False
    
    while True:
        enable_periodic_oled_update()
        FL_h = readCurrentHeading(adc_FL_A, adc_FL_B) if (adc_FL_A and adc_FL_B) else 0.0
        FR_h = readCurrentHeading(adc_FR_A, adc_FR_B) if (adc_FR_A and adc_FR_B) else 0.0
        BL_h = readCurrentHeading(adc_BL_A, adc_BL_B) if (adc_BL_A and adc_BL_B) else 0.0
        BR_h = readCurrentHeading(adc_BR_A, adc_BR_B) if (adc_BR_A and adc_BR_B) else 0.0

        ry = get_right_joystick_y_control_value()
        if ry > (joystick_center + joystick_deadzone):
            motor_speed = int((ry - joystick_center) * (254.0 / (1023 - joystick_center)))
        elif ry < (joystick_center - joystick_deadzone):
            motor_speed = -int((joystick_center - ry) * (254.0 / joystick_center))
        else:
            motor_speed = 0

        set_wheel_speed_struct(drive_FL, motor_speed)
        set_wheel_speed_struct(drive_FR, motor_speed)
        set_wheel_speed_struct(drive_BL, motor_speed)
        set_wheel_speed_struct(drive_BR, motor_speed)

        # Initialize variables for logging
        desired_heading = 0.0
        rotate_speed = 0

        if normal_drive_mode:
            rx = get_right_joystick_x_control_value()
            desired_heading = convert_right_joystick_to_heading_value(rx)
            set_wheel_heading_generic(steer_FL, desired_heading, FL_h)
            set_wheel_heading_generic(steer_FR, desired_heading, FR_h)
            set_wheel_heading_generic(steer_BL, desired_heading, BL_h)
            set_wheel_heading_generic(steer_BR, desired_heading, BR_h)

        if rotate_drive_mode:
            lx = get_left_joystick_x_control_value()
            if lx > (joystick_center + joystick_deadzone):
                rotate_speed = int((lx - joystick_center) * (254.0 / (1023 - joystick_center)))
            elif lx < (joystick_center - joystick_deadzone):
                rotate_speed = -int((joystick_center - lx) * (254.0 / joystick_center))
            else:
                rotate_speed = 0
            set_wheel_heading_generic(steer_FL, 45, FL_h)
            set_wheel_heading_generic(steer_FR, -45, FR_h)
            set_wheel_heading_generic(steer_BL, -45, BL_h)
            set_wheel_heading_generic(steer_BR, 45, BR_h)
            set_wheel_speed_struct(drive_FL, rotate_speed)
            set_wheel_speed_struct(drive_FR, -rotate_speed)
            set_wheel_speed_struct(drive_BL, rotate_speed)
            set_wheel_speed_struct(drive_BR, -rotate_speed)

        # Console logging for monitoring variables (every 2 seconds)
        current_time = time.ticks_ms()
        if time.ticks_diff(current_time, last_log_time) >= log_interval:
            last_log_time = current_time
            print("--- Main Loop Monitor ---")
            print("motor_speed: {}, desired_heading: {:.2f}".format(motor_speed, desired_heading))
            print("drive_FL: {}, drive_FR: {}, drive_BL: {}, drive_BR: {}".format(motor_speed, motor_speed, motor_speed, motor_speed))
            print("steer_FL: {:.2f}, steer_FR: {:.2f}, steer_BL: {:.2f}, steer_BR: {:.2f}".format(FL_h, FR_h, BL_h, BR_h))
            print("rotate_speed: {}, demonstration_mode: {}".format(rotate_speed, demonstration_mode))

        if demonstration_mode:
            run_steer_motor_struct(steer_FL, 80)
            run_steer_motor_struct(steer_FR, 80)
            run_steer_motor_struct(steer_BL, -80)
            run_steer_motor_struct(steer_BR, -80)
            set_wheel_speed_struct(drive_FL, 120)
            set_wheel_speed_struct(drive_FR, 120)
            set_wheel_speed_struct(drive_BL, 120)
            set_wheel_speed_struct(drive_BR, 120)
            time.sleep(1)
            stop_all_drive()
            stop_all_steer()
            demonstration_mode = False
            normal_drive_mode = True

        home_button_current = control_state['buttons']['home']
        if home_button_current and not home_button_last_state:
            print("Home pressed - starting homing")
            ok = driveMotorToHome(timeout_ms=8000)
            print("Homing done:", ok)
        home_button_last_state = home_button_current

        left_joystick_button_current = control_state['buttons']['leftJoystick']
        if left_joystick_button_current and not left_joystick_button_last_state:
            print("Left joystick button pressed")
        left_joystick_button_last_state = left_joystick_button_current

        right_joystick_button_current = control_state['buttons']['rightJoystick']
        if right_joystick_button_current and not right_joystick_button_last_state:
            print("Right joystick button pressed")
        right_joystick_button_last_state = right_joystick_button_current

        rotate_button_current = control_state['buttons']['rotate']
        if rotate_button_current and not rotate_button_last_state:
            rotate_drive_mode = not rotate_drive_mode
            normal_drive_mode = not rotate_drive_mode
            print("Rotate mode is now", rotate_drive_mode)
        rotate_button_last_state = rotate_button_current

        demo_button_current = control_state['buttons']['demo']
        if demo_button_current and not demo_button_last_state:
            demonstration_mode = True
            normal_drive_mode = False
            rotate_drive_mode = False
            print("Demo started")
        demo_button_last_state = demo_button_current

        time.sleep_ms(30)

ap = start_access_point()
_thread.start_new_thread(http_server_thread, ())

try:
    main_loop()
except Exception as e:
    print("Main loop exception:", e)
    stop_all_drive()
    stop_all_steer()
