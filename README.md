# SelfbalanceBot
A two wheeled bot that stays upright by itself. It senses how far it is falling then decides how hard to push and drives it's wheels under itself to catch the fall.
<br>
This is my first robotics learning project. I built this to understand sensors function, PID control and motor drivers. 
<br>
## Built with 
- ESP32
- TB6612FNG
- TT gear Motors
- 2S18650 pack
- MPU9250
  <br>

<img width="3024" height="4032" alt="final" src="https://github.com/user-attachments/assets/4b749cd6-6610-46e4-8501-1dbaf1781e22" />


# Bill Of Materials 
| Component | Quantity | Price/Unit (USD) | Purpose | Purchase Link |
|---|---|---|---|---|
| ESP32-WROOM-32 38-Pin Development Board | 1 | $4.18 | Main controller and PID processing | [Buy Here](https://robu.in/product/esp32-38pin-development-board-wifibluetooth-ultra-low-power-consumption-dual-core/) |
| MPU9250 9-Axis Module | 1 | $23.26 | Accelerometer and gyroscope for tilt sensing (magnetometer unused) | [Buy Here](https://robu.in/product/mpu9250-9-axis-attitude-gyro-accelerator-magnetometer-sensor-module-2/) |
| TB6612FNG Motor Driver | 1 | $1.54 | Dual H-bridge driver that switches battery current to the motors and sets their direction | [Buy Here](https://robu.in/product/motor-driver-tb6612fng-module-performance-ultra-small-volume-3-pi-matching-performance-ultra-l298n/) |
| 5V 200RPM TT DC Gear Motor 1:48 (Pair) | 1 | $4.65 | Geared motors that deliver the torque to correct the tilt | [Buy Here](https://robu.in/product/pair-5v-200rpm-microbit-tt-dc-motor-148/) |
| 65mm Robot Wheel for BO Motors | 2 | $0.28 | Drive wheels | [Buy Here](https://robu.in/product/robot-smart-car-wheel-tyre-bo-motor/) |
| 18650 Li-Ion Battery 3.7V 2000mAh | 1 | $0.64 | Rechargeable power source | [Buy Here](https://robocraze.com/products/3-7v-2000mah-18650-li-ion-battery?variant=41414818758809) |
| 18650 x 2 Battery Holder with Cover and On/Off Switch | 1 | $0.55 | Holds the cells safely and provides a power switch | [Buy Here](https://robu.in/product/18650-x-2-battery-holder-with-cover-and-on-off-switch/) |
| LM2596 DC-DC Buck Converter (Adjustable) | 1 | $0.45 | Steps battery voltage down to a stable 5V for the ESP32 | [Buy Here](https://robocraze.com/products/lm2596-dc-dc-buck-module?variant=40192693076121) |
| Male to Female Jumper Wires 40pcs 20cm | 1 | $0.46 | Prototyping connections between boards | [Buy Here](https://robu.in/product/male-to-female-jumper-wires-40pcs-20cm/) |
| Female to Female Dupont Wires 40pin 20cm | 1 | $1.63 | Prototyping connections between module headers | [Buy Here](https://robu.in/product/acebott-20cm-dupont-wire-40pin-female-to-female/) |
| 3-in-1 Soldering Iron Kit (25W iron + solder + flux) | 1 | $1.68 | Making permanent connections once the circuit is final | [Buy Here](https://www.amazon.in/dp/B07W32JT5N) |
| eSUN PLA Matte Deep Black 1.75mm Filament | 1 | $12.09 | 3D printing the chassis | [Buy Here](https://robu.in/product/esun-epla-gloss-1-75mm-3d-printing-filament-1kg/) |
| **Total** | | **$52.88** | | |

# Circuit Diagram
<img width="1084" height="583" alt="circuit diagram" src="https://github.com/user-attachments/assets/a24ae1f5-3e55-4ec8-8da7-45720ccc01db" />

**MPU9250 → ESP32**

| MPU9250 | ESP32 |
|---|---|
| VCC | 3.3V |
| GND | GND |
| SDA | GPIO TODO |
| SCL | GPIO TODO |

**TB6612FNG → ESP32**

| TB6612FNG | ESP32 |
|---|---|
| AIN1 / AIN2 | GPIO TODO / TODO |
| PWMA | GPIO TODO |
| BIN1 / BIN2 | GPIO TODO / TODO |
| PWMB | GPIO TODO |
| STBY | GPIO TODO |
| VCC (logic) | 3.3V |
| GND | GND |

**Motors:** left motor on A01/A02, right motor on B01/B02.


## Build steps

1. Print the chassis (f3d file in `/CAD`).
2. Test every module separately on a breadboard first: the IMU readings, then one motor at a time.
3. Wire everything following the tables above.
4. Assemble everything on the chassis.
5. Mount the MPU9250 **rigidly** and note its orientation. A wobbly sensor mount gives bad angle data.
6. Flash the firmware.
7. With the wheels off the ground, tilt the bot by hand. The wheels should spin in the direction of the tilt. If they spin the opposite way, flip the motor direction in the code.
8. Place it on a flat surface and tune.

## Tuning

Tune in this order and change one value at a time:

1. Set `Ki = 0` and `Kd = 0`.
2. Raise `Kp` until the bot reacts quickly and starts oscillating.
3. Add `Kd` until the oscillation calms down.
4. Add a small `Ki` only if the bot slowly leans to one side.
5. Adjust the target angle if the bot rests tilted because its weight is not centered.

PID values are specific to your weight, wheel size, motor and battery voltage. 

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Falls over instantly in one direction | Motor direction is reversed |
| Violent shaking | `Kp` too high, or `Kd` too low |
| Slow drift to one side | Tilt offset or imbalanced motors |
| Motors hum but do not move | PWM too low to overcome friction |
| Random resets | Power sag from motors; check buck converter and battery charge |
| Behaviour gets worse as the battery drains | Controller was tuned at a different voltage |

## CAD Models
<img width="1512" height="951" alt="Screenshot 2026-10-06 at 1 44 56 PM" src="https://github.com/user-attachments/assets/3561d677-6135-42ed-8238-65d7d8e80e26" />
<img width="3024" height="4032" alt="WhatsApp Image 2026-10-07 at 12 15 59" src="https://github.com/user-attachments/assets/08cb753c-6945-4b04-9437-d2b925e97d3e" />
<img width="1512" height="951" alt="Screenshot 2026-10-06 at 12 21 30 PM" src="https://github.com/user-attachments/assets/18d6ea5a-b114-4f40-aa8f-529b54069631" />

## Testing Video






https://github.com/user-attachments/assets/a5a6a393-e260-479e-8745-cf48acce4c8e








https://github.com/user-attachments/assets/3d98c6a5-224b-486f-9dc2-2ef70e6621a0







