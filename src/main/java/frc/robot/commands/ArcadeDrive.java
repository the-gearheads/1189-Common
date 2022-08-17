// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class ArcadeDrive extends CommandBase {
  private DoubleSupplier xAxis;
  private DriveTrain driveTrain;
  private DoubleSupplier rotAxis;

  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(DriveTrain driveTrain, DoubleSupplier xAxis, DoubleSupplier rotAxis) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.xAxis = xAxis;
    this.rotAxis = rotAxis;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xVal = xAxis.getAsDouble() * 1;
    double rotVal = rotAxis.getAsDouble() * 1;

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xVal, 0, rotVal);
    DifferentialDriveWheelSpeeds wheelSpeeds = driveTrain.toWheelSpeeds(chassisSpeeds);

    SmartDashboard.putNumber("xVal", xVal);
    SmartDashboard.putNumber("rotVal", rotVal);
    SmartDashboard.putNumber("Left Wheel Speed", wheelSpeeds.leftMetersPerSecond);
    SmartDashboard.putNumber("Right Wheel Speed", wheelSpeeds.rightMetersPerSecond);
    driveTrain.setRawSpeeds(wheelSpeeds);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
