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
    "buttons": {"home": False, "rotate": False, "demo": False}
}
control_lock = _thread.allocate_lock()

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

def LeftJoystickButtonPressed():
    control_lock.acquire()
    try:
        return bool(control_state['buttons']['home'])
    finally:
        control_lock.release()

def RightJoystickButtonPressed():
    control_lock.acquire()
    try:
        return bool(control_state['buttons']['rotate'])
    finally:
        control_lock.release()

def DemoButtonPressed():
    control_lock.acquire()
    try:
        return bool(control_state['buttons']['demo'])
    finally:
        control_lock.release()

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

HTML_PAGE = """HTTP/1.0 200 OK

<!doctype html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Mini SwerveDrive</title>
<style>body{{font-family:Arial;margin:0;background:#111;color:#eee}}.container{{display:flex;flex-direction:column;align-items:center;padding:10px}}.row{{display:flex;width:100%;justify-content:space-around;margin:8px 0}}.pad{{width:45vw;height:45vw;max-width:360px;max-height:360px;background:#222;border-radius:12px;touch-action:none;position:relative;overflow:hidden}}.knob{{position:absolute;width:24px;height:24px;border-radius:12px;background:#09f;transform:translate(-50%,-50%);left:50%;top:50%}}.btn{{background:#333;color:#fff;padding:12px 20px;border-radius:8px;margin:6px;border:none;font-size:16px}}.label{{text-align:center;margin-top:6px}}.small{{font-size:12px;color:#aaa}}</style>
</head>
<body>
<div class="container">
  <h2>Mini SwerveDrive</h2>
  <div class="row">
    <div id="leftPad" class="pad"><div id="leftKnob" class="knob"></div></div>
    <div id="rightPad" class="pad"><div id="rightKnob" class="knob"></div></div>
  </div>
  <div class="row">
    <button id="homeBtn" class="btn">Home</button>
    <button id="rotateBtn" class="btn">Rotate</button>
    <button id="demoBtn" class="btn">Demo</button>
  </div>
  <div class="label small">Connect to Wi-Fi: {ssid} (pw: {pw}) — open http://{apip}</div>
  <div class="label small" id="status">Status: connected</div>
</div>
<script>
const updateIntervalMs = 60;
function makePad(padId, knobId, stateKey) {{
  const pad = document.getElementById(padId);
  const knob = document.getElementById(knobId);
  let pointerId = null;
  function updateKnob(nx, ny){{ knob.style.left=(50+nx*50)+'%'; knob.style.top=(50-ny*50)+'%'; }}
  function down(e){{ e.preventDefault(); pointerId = e.pointerId || 1; move(e); }}
  function up(e){{ e.preventDefault(); pointerId=null; window[stateKey]={{x:0,y:0}}; updateKnob(0,0); }}
  function move(e){{
    if(pointerId !== null && e.pointerId && e.pointerId !== pointerId) return;
    const r = pad.getBoundingClientRect();
    let px = e.clientX || (e.touches && e.touches[0] && e.touches[0].clientX) || 0;
    let py = e.clientY || (e.touches && e.touches[0] && e.touches[0].clientY) || 0;
    let dx = px - (r.left + r.width/2), dy = py - (r.top + r.height/2);
    let maxR = Math.min(r.width, r.height)/2;
    let nx = dx/maxR; if(nx>1) nx=1; if(nx<-1) nx=-1;
    let ny = dy/maxR; if(ny>1) ny=1; if(ny<-1) ny=-1;
    // FIXED: Invert ny so up = +1, down = -1, and knob moves up when you move up
    ny = -ny;
    window[stateKey] = {{x:nx,y:ny}};
    updateKnob(nx, ny);
  }}
  pad.addEventListener('pointerdown', down);
  pad.addEventListener('pointermove', move);
  pad.addEventListener('pointerup', up);
  pad.addEventListener('pointercancel', up);
  pad.addEventListener('touchstart', down, {{passive:false}});
  pad.addEventListener('touchmove', move, {{passive:false}});
  pad.addEventListener('touchend', up, {{passive:false}});
  window[stateKey] = {{x:0,y:0}};
}}
makePad('leftPad','leftKnob','leftState');
makePad('rightPad','rightKnob','rightState');

let buttons = {{home:false, rotate:false, demo:false}};
document.getElementById('homeBtn').addEventListener('click', ()=>{{ buttons.home = !buttons.home; document.getElementById('homeBtn').style.background = buttons.home ? '#0a0' : '#333'; }});
document.getElementById('rotateBtn').addEventListener('click', ()=>{{ buttons.rotate = !buttons.rotate; document.getElementById('rotateBtn').style.background = buttons.rotate ? '#0a0' : '#333'; }});
document.getElementById('demoBtn').addEventListener('click', ()=>{{ buttons.demo = !buttons.demo; document.getElementById('demoBtn').style.background = buttons.demo ? '#0a0' : '#333'; }});

async function sendLoop(){{
  const url = '/control';
  while(true){{
    const payload = {{ left: window.leftState || {{x:0,y:0}}, right: window.rightState || {{x:0,y:0}}, buttons: buttons }};
    try {{
      if(navigator.sendBeacon) navigator.sendBeacon(url, JSON.stringify(payload));
      else await fetch(url, {{method:'POST', body: JSON.stringify(payload)}});
    }} catch(e) {{ document.getElementById('status').innerText='Status: network error'; }}
    await new Promise(r=>setTimeout(r, updateIntervalMs));
  }}
}}
sendLoop();
</script>
</body>
</html>
""".format(ssid=AP_SSID, pw=AP_PASSWORD, apip=AP_STATIC_IP)

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

def http_server_thread():
    addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(addr)
    s.listen(1)
    print("HTTP server listening on", addr)
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
            content_length = 0
            while True:
                header = cl_file.readline()
                if not header or header == b'\r\n':
                    break
                try:
                    h = header.decode()
                    if 'Content-Length:' in h:
                        content_length = int(h.split(':')[1].strip())
                except Exception:
                    pass
            if method == 'GET':
                cl.send(HTML_PAGE.encode('utf-8'))
            elif method == 'POST' and path.startswith('/control'):
                body = cl_file.read(content_length) if content_length else b''
                try:
                    data = ujson.loads(body)
                    control_lock.acquire()
                    try:
                        if 'left' in data:
                            control_state['left']['x'] = float(data['left'].get('x', 0.0))
                            control_state['left']['y'] = float(data['left'].get('y', 0.0))
                        if 'right' in data:
                            control_state['right']['x'] = float(data['right'].get('x', 0.0))
                            control_state['right']['y'] = float(data['right'].get('y', 0.0))
                        if 'buttons' in data:
                            for b in ('home', 'rotate', 'demo'):
                                control_state['buttons'][b] = bool(data['buttons'].get(b, False))
                    finally:
                        control_lock.release()
                    cl.send("HTTP/1.0 200 OK\r\n\r\nOK")
                except Exception:
                    cl.send("HTTP/1.0 400 Bad Request\r\n\r\n")
            else:
                cl.send("HTTP/1.0 404 Not Found\r\n\r\n")
            cl.close()
        except Exception as e:
            try:
                cl.close()
            except:
                pass
            time.sleep_ms(50)

normal_drive_mode = True
rotate_drive_mode = False
demonstration_mode = False

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

def main_loop():
    global normal_drive_mode, rotate_drive_mode, demonstration_mode
    joystick_center = 510
    joystick_deadzone = 5
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

        if LeftJoystickButtonPressed():
            print("Home pressed - starting homing")
            ok = driveMotorToHome(timeout_ms=8000)
            print("Homing done:", ok)

        if RightJoystickButtonPressed():
            rotate_drive_mode = not rotate_drive_mode
            normal_drive_mode = not rotate_drive_mode
            print("Rotate mode is now", rotate_drive_mode)
            time.sleep_ms(300)

        if DemoButtonPressed():
            demonstration_mode = True
            normal_drive_mode = False
            rotate_drive_mode = False
            print("Demo started")
            time.sleep_ms(300)

        time.sleep_ms(30)

ap = start_access_point()
_thread.start_new_thread(http_server_thread, ())

try:
    main_loop()
except Exception as e:
    print("Main loop exception:", e)
    stop_all_drive()
    stop_all_steer()
