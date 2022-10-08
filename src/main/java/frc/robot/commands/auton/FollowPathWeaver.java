package frc.robot.commands.auton;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class FollowPathWeaver extends CommandBase {

  private DriveTrain driveTrain;
  private Trajectory trajectory;
  private String trajectoryJSON;
  private Timer timer;
  private RamseteController ramsete;


  public FollowPathWeaver(DriveTrain driveTrain, String trajectoryJSON){
    this.driveTrain = driveTrain;
    this.trajectoryJSON = "paths/" + trajectoryJSON + ".wpilib.json";
    this.timer = new Timer();
    this.ramsete = new RamseteController();
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.trajectory = new Trajectory();
    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(this.trajectoryJSON);
        this.trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + this.trajectoryJSON, ex.getStackTrace());
        cancel();
    }

    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.updateOdometry();
    Trajectory.State goal = trajectory.sample(timer.get());
    ChassisSpeeds chassisSpeeds = ramsete.calculate(driveTrain.getPose(), goal);
    DifferentialDriveWheelSpeeds wheelSpeeds = driveTrain.toWheelSpeeds(chassisSpeeds);
    DifferentialDriveWheelSpeeds ff = driveTrain.calculateFF(wheelSpeeds);
    
    driveTrain.setRawSpeeds(ff);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    driveTrain.setRawSpeeds(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return trajectory.getTotalTimeSeconds() < timer.get();
  }
}
