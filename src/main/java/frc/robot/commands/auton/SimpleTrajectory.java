// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class SimpleTrajectory extends CommandBase {
  private DriveTrain driveTrain;
  private Trajectory trajectory;
  RamseteController ramsete = new RamseteController();
  private Timer timer = new Timer();
  private Pose2d initialPos;
  private List<Translation2d> interiorWaypoints;
  private Pose2d finalPos;
  private boolean isBackward = false;

  public SimpleTrajectory(DriveTrain driveTrain, Pose2d initialPos, Pose2d finalPos, boolean isBackward) {
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.initialPos        = initialPos;
    this.interiorWaypoints = new ArrayList<>();
    this.finalPos          = finalPos;
    this.isBackward        = isBackward;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setPose(initialPos);
    
    TrajectoryConfig config = new TrajectoryConfig(2, 1.5);//*2 and *1.5
    config.setReversed(isBackward);
    SmartDashboard.putString("Auton Input", "Init Pos: " + initialPos + "; finalPos: " + finalPos);
    trajectory = TrajectoryGenerator.generateTrajectory(
        initialPos,
        interiorWaypoints,
        finalPos,
        config);
    TrajectoryGenerator.generateTrajectory(
        initialPos,
        interiorWaypoints,
        finalPos,
        config);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.updateOdometry();
    Trajectory.State goal = trajectory.sample(timer.get());
    ChassisSpeeds ramseteValue = ramsete.calculate(driveTrain.getPose(), goal);
    DifferentialDriveWheelSpeeds wheelSpeeds = driveTrain.toWheelSpeeds(ramseteValue);
    driveTrain.setRawSpeeds(wheelSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return  timer.get() >= trajectory.getTotalTimeSeconds();
  }
}
