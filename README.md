#Wallie

![image](shot.jpg "Wallie selfie")

## A Collection of Sketches

This is a collection of sketches for a Wallie-like two-wheeled bot - i.e. a surface moving robot that interacts with the world - that is controlled using the ubiquitous Arduino Uno microcontroller board.

Each subfolder contains a sketch and some documentation about the robot config the sketch ran on.

All examples are actual sketches that were used with a real robots. Each is basically a snap shot of my progression with learning about robotics.

## Using the Sketches

If you want to adapt any of these sketches to your own projects keep in mind that you will most likely need to make modifications to get the code to work for your specific hardware. I try to abstract out as much of the device specific code as possible so it should be relatively wasy to adapt the code - after all I'll probably need to adapt the code myself as I change systems over time.

Currently the following systems are used for most of the examples:

* The [Adafruit Motor V2 Shield](http://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino) is used to drive the wheel motors.
* El Cheapo HC-SR04 Sonic Range Sensor

##The Sketches

 * **SpinBot** - The "Hello World!"" of robotic movement. Simply spins in place. (Like you need a sketch for that)
 
 * **WallBot V1** - Uses sonic sensor to avoid crashing into walls.
 
##Up Coming Projects/Hardware

 * Add Quadrature encoders
 * Add inertia sensors
 * Add compass