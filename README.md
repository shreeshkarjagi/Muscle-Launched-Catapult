# Muscle-Launched-Catapult
Innovative Physiotherapy: Muscle-controlled Catapult using Myoware &amp; Servo


## Introduction

This project, created by the Team: Muscle Boyz (Andrew Tejada, Eric Zavala, Jason Kong, Shreesh Karjagi, Utkarsh Kumar) during Idea Hacks LA 2023, aims to revolutionize physiotherapy by using cutting-edge technology to develop a muscle-controlled catapult. By utilizing a Myoware muscle sensor and a servo motor, the patient's muscle activity is used to launch the catapult. Not only is this an engaging and dynamic way to rehab injuries and improve muscle strength, it also empowers the patient to take control of their own therapy.

## Motivation

Traditional physiotherapy can be tedious and unengaging for patients, leading to poor adherence and suboptimal outcomes. By creating a dynamic and interactive tool that utilizes the patient's own muscle power, we hope to increase patient engagement and motivation in their therapy. Additionally, the muscle-controlled aspect of the catapult provides a unique way to specifically target and strengthen specific muscle groups.

## Solution

Our solution is a muscle-controlled catapult that utilizes a Myoware muscle sensor to measure the electrical activity of a patient's muscle. This data is then processed by an Arduino microcontroller, which sends PWM commands to a servo motor. The servo motor actuates the catapult, launching it based on the patient's muscle activity.

## Implementation

### Muscle Sensing
The Myoware muscle sensor is placed on the patient's muscle and connected to an Arduino microcontroller. The sensor measures the electrical activity of the muscle and sends the data to the microcontroller.

### Arduino Signal Processing
The Arduino microcontroller processes the muscle activity data and converts it into PWM commands for the servo motor using the code in [EMG_signalactuated_Servo.ino](https://github.com/shreeshkarjagi/Muscle-Launched-Catapult/blob/main/EMG_signalactuated_Servo.ino). This code acquires the signal, filters it, and compares it to a pre-set threshold to actuate the servo.


### PWM Commands
The PWM commands, found in [EMG_signalactuated_Servo.ino](https://github.com/shreeshkarjagi/Muscle-Launched-Catapult/blob/main/EMG_signalactuated_Servo.ino), are sent to the servo motor, which actuates the catapult, launching it based on the patient's muscle activity.

## Bill of Materials

- Myoware muscle sensor kit
- Arduino microcontroller
- Servo motor
- 3D Catapult mechanism
- Wires and connectors

## 3D Catapult mechanism

![3D Printed Catapult 1](https://github.com/shreeshkarjagi/Muscle-Launched-Catapult/blob/main/Catapult_3Dprint1.jpg)
![3D Printed Catapult 2](https://github.com/shreeshkarjagi/Muscle-Launched-Catapult/blob/main/Catapult_3Dprint2.jpg)
![3D Printed Catapult 3](https://github.com/shreeshkarjagi/Muscle-Launched-Catapult/blob/main/Catapult_3Dprint3.jpg)

## Demo Video

![Demo]([https://github.com/shreeshkarjagi/Muscle-Launched-Catapult/blob/main/Catapult_3Dprint3.jpg](https://github.com/shreeshkarjagi/Muscle-Launched-Catapult/blob/main/Demo.mov))

## Future Work

- Incorporating additional sensors, such as accelerometers, to track the patient's progress and provide feedback.
- Developing a mobile app to track patients' progress and provide personalized therapy plans.
- Investigating the use of the muscle-controlled catapult for other applications, such as physical rehabilitation for stroke patients.

We hope that this project serves as a proof of concept for the potential of using muscle-controlled technology in physiotherapy and other rehabilitation settings. We are excited to see how this technology can be further developed and utilized in the future.

