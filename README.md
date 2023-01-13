# 2DX4_FinalProject
## Table of contents

- [Project Overview](#project-overview)
  - [Project Description](#general-description)
  - [Block Diagram](#block-diagram)
- [Our process](#our-process)
  - [How we built it](#how-we-built-it)
  - [Challenges we ran into](#challenges-we-ran-into)
  - [Accomplishments that we're proud of](#accomplishments-that-we're-proud-of)
  
## Project Overview

### General Description
The 2DX Final Spatial Mapping project is a 3D scanning device that can capture distances
in several 360-degree planes along an orthogonal axis and then process the data to create a 3D
visualization of the mapped place. The system consists of a microcontroller, a time-of-flight
sensor, and a stepper motor. The microcontroller is responsible for managing all system
operations, excluding the 3D visualization component. Moreover, it delivers power and
configures other parts of the system (motor, sensor, stop button, etc.) and transmits all the
recorded data to an external device through serial communication.

The stepper motor provides a 360-degree range of motion for the device, allowing the
mounted time-of-flight sensor to capture distance measurements in the vertical plane. The time of-flight sensor generates pulses of infrared light and measures the time it takes for the emitted laser pulses to be reflected to the detector to determine distances to objects/surfaces. The sensor
then calculates the distance using this timing data based on its configuration settings and transmits
it to the microcontroller through the I2C communication. This component is mounted to the
stepper motor and takes a measurement every 45 degrees. This event occurs when the onboard button is pressed, and the stepper motor is rotated to capture the 360-degree distance measurements. 

The system is connected to a PC capable of serial communication via USB, which is used
to run the included Python script (data_visualization.py). Moreover, the microcontroller sends
status messages and distance measurements to the computer via UART. So, the data can then be
read, converted into x-y-z coordinates, and then visualized using a Python script.

### Block Diagram
![Desktop-version](client/my-app/public/images/LandingPage.png)

