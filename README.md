# Servo-Controller 2.0

Arduino Nano based Servo-Controller used in my robotic projects after 
installation of new servos. Before starting the designated control software
for the whole robot, this sketch on an Arduino Nano with its related Shield
allows testing each servo one-by-one for its allowed movement radius. Up to
4 servos are supported. 
The status is displayed on a small 20x2 character LCD, showing the current
settings with its lower and upper limits of action radius. If the onboard
LED is on, servos are ready to move and follow any change in position. By
pressing the button ModeControl at any time, all servos stop immediatly and
are released from control (no PWM signal any more). This does protect the 
robot mechanics, if the servo turns in the wrong direction or is going to
move beyond the mechanical boundaries of a joint.
 
![Servo-Controller 2.0](./doc/ServoControllerV2.png)

Release Notes:
	
Version 2.0 - 29.08.2018

	* Initial versions
