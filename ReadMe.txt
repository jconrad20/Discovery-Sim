This project aims to create an interactive game to simulate landing the Discovery Space Shuttle after it enters Earth’s atmosphere. The user controls the shuttle’s movement (pitch and roll), using a joystick. The roll is also modeled on the shuttle itself through the Elevons. 
The game begins with the shuttle flying at 17,400 mph and gradually slows to 250 mph (break in speed to landing speed) over 60 seconds. The speed affects the range of motion on the Elevons, preventing them from breaking off at high speeds or inducing instability. The motor speed is controlled on the joystick position (increasing with further movement from center) modeling the feedback felt by the pilot during operation. 
The position is always moved relative to the last commanded position as the shuttle is inherently unstable, built as a glider. When the joystick is in the null position the Shuttle continues the previous input attitude. 
The setup includes an Arduino Uno, LEGO Model 10283, two L298N motor drivers, eight pin joints, one MicroG servo, four DC Linear Actuators, an LCD screen, and a power supply up to 12V. 
Limitations of this build include no positioning of the shuttle in the physical world (Ex. Incorporating Microsoft Flight Sim) and hence no crash criteria considered.
Necessary libraries include Wire.h, Arduino.h, Servo.h, and LiquidCrystal_I2C.h

