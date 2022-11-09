// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class CorrectPoseWithVision extends CommandBase {
  private DriveTrain drivetrain;
  private Vision vision;

  /** Creates a new CorrectPoseWithVision. */
  public CorrectPoseWithVision(Vision vision, DriveTrain drivetrain) {
    this.vision = vision;
    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("is working",true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(vision.hasTargets()){
      Pose3d robotPos3d=vision.getRobotPosFromBestTarget();
      Pose2d robotPos2d=robotPos3d.toPose2d();
      robotPos2d=new Pose2d(robotPos2d.getTranslation(), drivetrain.getPose().getRotation());
      drivetrain.addVisionMeasurement(robotPos2d, Timer.getFPGATimestamp() - vision.getLatency()/1000.0);
    }
    //oh wow life changing code!
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
