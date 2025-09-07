# Mini-SwerveDrive-1895

Mini-SwerveDrive-1895 is a MicroPython program for an ESP32 (ESP-WROOM) that hosts a Wi‑Fi access point and serves a simple web page with two virtual joysticks and three buttons (Home, Rotate, Demo). The web UI controls a 4-wheel swerve-drive robot using L298N (ZX-040) motor drivers and optional wheel-heading analog sensors and an SSD1306 OLED.

This repository currently contains:
- main.py — the MicroPython script that starts a Wi‑Fi AP, serves the web UI, reads control input, and drives the motors.

Important safety note
- Test with motors disconnected first. Verify the web UI updates and printed/log output before connecting motors.
- L298N motor supply must be external to the ESP32’s 3.3V and share a common ground.
- Use low-power/limit PWM (or remove motor power) while debugging to avoid unexpected motion.

Features
- ESP32 acts as Wi‑Fi Access Point (AP) and serves a touch-friendly joystick page at http://192.168.10.1
- Two touchscreen joysticks (left/right) and three toggle buttons: Home, Rotate, Demo
- Basic integration for:
  - Drive motors (spin wheels — actually move the robot)
  - Steering motors (rotate wheel modules — change heading)
  - Optional analog wheel heading sensors (ADC1 pins)
  - Optional SSD1306 OLED (I2C)
  - Optional IMU detection (common MPU6050 @ 0x68)
- Homing, rotate-in-place, and demo modes built-in (see code comments)

Default AP configuration (from main.py)
- SSID: Mini-RC-SwerveDrive-AP
- Password: FRC_TEAM1895
- AP static IP: 192.168.10.1 (open http://192.168.10.1 on a connected phone or laptop)

Default pin mapping (edit main.py to match your wiring)
- ADC / wheel-heading sensors (use ADC1 pins — required when Wi‑Fi is active)
  - FL_HeadingSensor_A_pin = 32
  - FL_HeadingSensor_B_pin = 33
  - FR_HeadingSensor_A_pin = 34
  - FR_HeadingSensor_B_pin = 35
  - BL_HeadingSensor_A_pin = 36
  - BL_HeadingSensor_B_pin = 39
  - BR_HeadingSensor_A_pin = 37  (set to None if unavailable)
  - BR_HeadingSensor_B_pin = 38  (set to None if unavailable)

- I2C (OLED / IMU)
  - SDA = 21
  - SCL = 22
  - SSD1306 assumed at 0x3C, 128x32 (optional)

- Drive motors (spin the wheels — move the robot)
  - FL_drive = (dirA=16, dirB=17, pwm=25)
  - FR_drive = (dirA=18, dirB=19, pwm=14)
  - BL_drive = (dirA=26, dirB=27, pwm=12)
  - BR_drive = (dirA=13, dirB=15, pwm=2)

- Steering motors (rotate wheel modules — change heading)
  - FL_steer = (dirA=4, dirB=5, pwm=32)
  - FR_steer = (dirA=33, dirB=34, pwm=14)
  - BL_steer = (dirA=35, dirB=36, pwm=27)
  - BR_steer = (dirA=39, dirB=37, pwm=13)

Notes about pins
- The defaults in main.py are illustrative. Do not use the defaults on the board without checking for pin conflicts.
- ADC1 pins (GPIO32..GPIO39) must be used for analog wheel sensors while Wi‑Fi is active. ADC2 pins are shared with Wi‑Fi and will be unreliable.
- Some GPIOs are input-only or used for boot strapping — avoid using pins that change boot behavior (GPIO0, GPIO2, GPIO15, etc.) unless you understand the consequences.
- Ensure you do not assign the same GPIO to two different functions.

Dependencies
- MicroPython firmware for ESP32 (latest stable build recommended).
- main.py uses builtin modules: network, socket, _thread, time, ujson, machine.
- Optional: ssd1306 MicroPython driver (if you want OLED output). Place ssd1306.py on the ESP32 with your main.py (many MicroPython builds include this).
- Optional: add an IMU driver for your exact 9‑DOF sensor if you want full sensor integration.

How to flash main.py
- Use one of these tools to copy main.py to the device:
  - Thonny: open main.py and save to device (Recommended for beginners).
  - ampy (Adafruit-ampy): ampy --port /dev/ttyUSB0 put main.py
  - rshell / mpfshell / mpremote: mpremote cp main.py :/main.py
- Reboot the ESP32 after copying main.py.

How to use
1. Power the ESP32 (and the separate motor supply for the L298Ns). Ensure grounds are common.
2. Connect a phone or laptop to Wi‑Fi SSID: Mini-RC-SwerveDrive-AP (password: FRC_TEAM1895).
3. Open http://192.168.10.1 in your browser on the connected device.
4. Use left/right virtual joysticks and toggle buttons:
   - Left joystick: used in rotate-drive mode for rotation speed (depending on code).
   - Right joystick: Y controls drive speed; X controls wheel heading in normal drive mode.
   - Buttons:
     - Home: runs homing routine (if wheel heading sensors are available & configured).
     - Rotate: toggles rotate-in-place mode (steer headings fixed; wheels spin to rotate).
     - Demo: runs a simple demo sequence (brief movement/steering pattern).
5. Monitor the serial REPL output (USB serial) to see status and debugging prints.

Behavior and limitations
- The web UI posts control state frequently (about every 60 ms). This is implemented with HTTP POST (sendBeacon/fetch). WebSockets would be lower latency but increase implementation complexity.
- Steering logic in main.py is a simplified port of the original swerve logic and may require tuning to your exact hardware (steering speeds, heading offsets).
- Homing requires wheel heading analog sensors and assigns offsets for each wheel; you must calibrate offsets and verify sensor wiring and orientation.
- PWM duty scaling in MicroPython can differ by build. The helpers in main.py try duty() then duty_u16(); adjust if your firmware differs.

Troubleshooting
- Motors do not spin:
  - Confirm motor power supply is present and shared ground.
  - Confirm L298N EN pin is the PWM pin in the mapping. On some L298N boards EN is separate from IN1/IN2.
  - Verify wiring and that you are not using boot pins that force inputs at boot.
- No ADC readings or wrong values:
  - Ensure you use ADC1 pins (GPIO32..39) when Wi‑Fi is used.
  - Check that ADC pins are not also used for steering PWMs in your mapping.
- OLED not showing text:
  - Ensure ssd1306 driver is available on the board.
  - Verify I2C wiring (SDA/SCL) and device address (0x3C common).
- HTTP server errors / blank page:
  - Connect to AP, open http://192.168.10.1 in browser. If you see 404 or disconnects, check serial logs and ensure the HTTP server thread started without exceptions.

Customizing
- Edit the PIN MAP section at top of main.py for your exact wiring.
- Change AP_SSID / AP_PASSWORD / AP_STATIC_IP as needed.
- Tweak steering base speeds, deadzones, and heading tolerances inside the swerve logic for smooth control.

License
- MIT License (add a LICENSE file if you want the repository to include it). If you want a different license, replace it accordingly.
