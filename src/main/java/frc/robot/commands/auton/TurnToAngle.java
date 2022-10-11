// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import java.net.Authenticator.RequestorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class TurnToAngle extends CommandBase {
  private double requestedAngle;
  private DriveTrain driveTrain;
  private PIDController rotController = new PIDController(0.05, 0, 0);
  private int satisfactionNumber = 0;
  double maxRotSpeed = Constants.DRIVE.MAX_ROT_VEL * (0.5);
  double minRotSpeed = Constants.DRIVE.MAX_ROT_VEL * (0.03);

  /** Creates a new TurnToAngleC. */
  public TurnToAngle(DriveTrain driveTrain, double requestedAngle) {
    this.requestedAngle = requestedAngle;
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("turning", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Rotational Speed Calculations
    double currentAngle = driveTrain.getPose().getRotation().getDegrees();
    SmartDashboard.putNumber("Current Rot", currentAngle);
    SmartDashboard.putNumber("Wanted Rot", requestedAngle);

    double rotSpeed = rotController.calculate(currentAngle, requestedAngle);
    double rotDirection = rotSpeed > 0? 1 : -1;
    rotSpeed = Math.abs(rotSpeed);

    if(Math.abs(currentAngle - requestedAngle) < 2){
      rotSpeed = 0;
    }else{
      rotSpeed = Math.max(Math.min(rotSpeed, maxRotSpeed), minRotSpeed);
    }

    driveTrain.setRawSpeeds(0, rotSpeed * rotDirection);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setRawSpeeds(0,0);
    SmartDashboard.putBoolean("turning", false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentAngle = driveTrain.getPose().getRotation().getDegrees();
    if(Math.abs(currentAngle - requestedAngle) < 4){
      satisfactionNumber++;
    }else{
      satisfactionNumber=0;
    }
    return satisfactionNumber > 5;
  }
}

