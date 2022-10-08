// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveTrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
    driveTrain.setIdleMode(true);
    driveTrain.setRampRate(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xInput = xAxis.getAsDouble();
    double rotInput = rotAxis.getAsDouble();

    xInput = Math.copySign(xInput*xInput, xInput);
    rotInput = Math.copySign(rotInput*rotInput, rotInput);

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xInput, 0, rotInput);
    DifferentialDriveWheelSpeeds wheelSpeeds = driveTrain.toWheelSpeeds(chassisSpeeds);
    
    double largestVal = Math.max(Math.abs(wheelSpeeds.leftMetersPerSecond), Math.abs(wheelSpeeds.rightMetersPerSecond));
    if(largestVal > 1){
      wheelSpeeds.leftMetersPerSecond/=largestVal;
      wheelSpeeds.rightMetersPerSecond/=largestVal;
    }
    SmartDashboard.putNumber("left wheel", wheelSpeeds.leftMetersPerSecond);
    SmartDashboard.putNumber("right wheel", wheelSpeeds.rightMetersPerSecond);
    wheelSpeeds.leftMetersPerSecond*=Constants.Drive.MAX_X_VEL;
    wheelSpeeds.rightMetersPerSecond*=Constants.Drive.MAX_X_VEL;
    SmartDashboard.putNumber("left wheel after", wheelSpeeds.leftMetersPerSecond);
    SmartDashboard.putNumber("right wheel after", wheelSpeeds.rightMetersPerSecond);
    var ff = driveTrain.calculateFF(wheelSpeeds);
    SmartDashboard.putNumber("left FF", ff.leftMetersPerSecond);
    SmartDashboard.putNumber("right FF", ff.rightMetersPerSecond);
    
    driveTrain.setRawSpeeds(ff);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setRawSpeeds(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
