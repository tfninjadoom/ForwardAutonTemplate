## What
Template for easy autonomous that moves the robot a very short distance forward.  
Contains custom (time-dependent) and LemLib (odometry-dependent) forward autonomous functions that can be switched with an enum in `src/main.cpp`.  

## Why
This code was created for the purpose of ensuring that we have an easy way to create this simple Autonomous across a wide variety/range of robots, regardless of their drivetrain configuration and other design choices.

## How To Use  
Update the port values within the motor group and change the wheel size depending on the robot you are using, then the code should compile and run.  
To switch between the different functions, change the value of `int AUTON_SELECT` to an `enum AutonSelect` value in `src/main.cpp`.
