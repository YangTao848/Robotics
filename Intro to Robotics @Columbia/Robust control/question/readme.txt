
singleJointManipulator.m:
A class for defining objects with a single link rotating around a revolute joint.
It uses Matlab OOP. To create an object, one needs to pass the static parameters (moment of inertia about COM, mass, link length, and distance between the center of revolute joint and COM) to the constructor.
The variables (joint angle, input torque, controller gains, and model parameters) are private properties which can only be defined and accessed by calling the corresponding setter and getter.
The class has functions of computing kinematics, dynamics, motion planning using 3rd order polynomial and control using computed torque method.


script.m:
This file is a script file, where you can find the ways of using functions defined in the class file.