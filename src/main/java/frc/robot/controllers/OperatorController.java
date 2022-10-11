package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
public class OperatorController  {
    Joystick controller;

    public OperatorController(int port) {
        controller = new Joystick(port);
    }
}
