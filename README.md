#Wallie

![image](shot.jpg "Wallie selfie")

## A Collection of Sketches

This is a collection of sketches for a Wallie-like two-wheeled bot - i.e. a surface moving robot that interacts with the world - that is controlled using the ubiquitous Arduino Uno microcontroller board.

Each subfolder contains a sketch and some documentation about the robot config the sketch ran on.

All examples are actual sketches that were used with a real robot. Each is basically a snap shot of my progression with developing Wallie.

## Using the Sketches

If you want to adapt any of these sketches to your own projects keep in mind that you will most likely need to make modifications to get the code to work for your specific hardware. I try to abstract out as much of the device specific code as possible so it should be relatively easy to adapt the code - after all I'll probably need to adapt the code myself as I change systems over time.

Currently the following systems/hardware are used for most of the examples:

* The [Adafruit Motor V2 Shield](http://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino) is used to drive the wheel motors.
* El Cheapo HC-SR04 Sonic Range Sensor
* TowerPro SG-50 Micro Servos

##The Sketches

 * **SpinBot** - The "Hello World!"" of robotic movement. Simply spins in place. (Like you need a sketch for that)
 
 * **WallBot V1** - Uses sonic sensor to avoid crashing into walls.
 
 * **WallBot V2** - Sonic sensor is mounted to a servo.
 
##Up Coming Projects/Hardware

 * Add Quadrature encoders
 * Add inertia sensors
 * Add compass
 * Moving to an RTOS kernel
 * Replace motor shield with dedicated motor controller
 * Add IR LEDs to provide edge detection, i.e. avoid falling off a table.
 * Wireless communication and control