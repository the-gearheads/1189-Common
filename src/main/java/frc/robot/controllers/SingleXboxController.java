package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.controllers.Controllers.ControllerMode;

public class SingleXboxController  {
    XboxController controller;

    public SingleXboxController(int port) {
        controller = new XboxController(port);
    }

    public double getMoveAxis() {
        return -MathUtil.applyDeadband(controller.getLeftY(), Constants.Controller.DRIVE_DEADBAND);
    }
    public double getRotateAxis() {
        return MathUtil.applyDeadband(controller.getRightX(), Constants.Controller.ROTATE_DEADBAND); // delete and uncomment next line
        // return MathUtil.applyDeadband(controller.getRightX(), Constants.Controller.ROTATE_DEADBAND);
    }

    public Button getZeroOdometryButton(ControllerMode mode){
        if(Controllers.mode.equals(mode))
            return new JoystickButton(controller, XboxController.Button.kA.value);
        else   
            return new Button(()->false);
    }
    public Button getResetOdometryButton(ControllerMode mode){
        if(Controllers.mode.equals(mode))
            return new JoystickButton(controller, XboxController.Button.kY.value);
        else
            return new Button(()->false);
    }
}
