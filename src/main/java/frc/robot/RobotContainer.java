// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.xml.stream.util.XMLEventConsumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.auton.FollowPathWeaver;
import frc.robot.commands.auton.SimpleTrajectory;
import frc.robot.commands.driveTrain.FFCharacterizer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.controllers.Controllers;
import frc.robot.controllers.Controllers.ControllerMode;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain driveTrain = new DriveTrain();
  private final XboxController controller = new XboxController(0);
  // private final Vision vision = new Vision();
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings(){
    Controllers.mode = ControllerMode.TESTING;
    // Odometry Testing Mode
    new JoystickButton(controller, 1).whenPressed(new InstantCommand(()->{
      SmartDashboard.putBoolean("THIS WORKED", true);
      driveTrain.setPosition(Constants.Drive.ZERO_POSITION);
    }));
    new JoystickButton(controller, 3).whenPressed(new InstantCommand(()->{
      SmartDashboard.putBoolean("THIS WORKED", true);
      driveTrain.setPosition(Constants.Drive.START_POSITION);
    }));
    new JoystickButton(controller, 4).whenPressed(new FollowPathWeaver(driveTrain, "StartToHuman"));
    new JoystickButton(controller, 2).whenPressed(new FollowPathWeaver(driveTrain, "HumanToStart"));

  }

  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }

  public void updateControllers() {
    // Do nothing if controller layout hasn't changed.
    if(!Controllers.didControllersChange()) return; 
    System.out.println("Updating controller layout");

    // Clear buttons
    CommandScheduler.getInstance().clearButtons();

    // Find new controllers
    Controllers.updateActiveControllerInstance();

    configureButtonBindings();
  }
}
