# Feedback Linearization Controller for AUV.
(AUV = Autonomous Underwater Vehicle)

## Introduction and Installation
This repository implement a Feedback Linearization Controller for AUV in Python with ROS middleware. 
This Controller has been implemented in a faithful simulation of AUV designed in MDM Lab of University of Florence.
With a modular property of ROS, old PID controller has been replaced by this new inverse-dynamic based controller. 
Reference for this project is: https://www.fossen.biz/wiley/ed1/Ch13.pdf

Requirements are: http://wiki.ros.org/ROS/Tutorials and https://www.python.org/download/releases/2.7/

## Feedback Linearization
This control guarantees asymptotic stability with dynamic of error expressed in body-fixed frame.
<img src="./design/dinamica_inversa.svg">

## UML
Generic function of Controller:
<img src="./design/UML.svg">
- ControlNode: RosNode that interface directly with ROS platform to preleve reference.
- InverseDynamicController: Class that implement mathematical operations for Controller.
- PI: Class that implement Proportional-Integral component of velocity error dynamic.
- VehicleModelAUV: Class that implement the dynamic model for the AUV considered.

### Class Diagram 
<img src="./design/class_diagram.svg">

## Reference Scheme
Using page 54 [1] of the Reference https://www.fossen.biz/wiley/ed1/Ch13.pdf, "Decoupling in the BODY Frame
(Velocity Control)", have to be careful to how the references are expressed, if <body> or <ned>. This project needs
reference in body-fixed frame [1].

<img src="./design/scheme.svg">

## Global Implementation
The follow provides a complete vision of project and how to integrate it.

<img src="./design/Implementation.svg">

