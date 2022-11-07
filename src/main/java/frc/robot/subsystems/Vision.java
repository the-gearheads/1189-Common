// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.ConnectionInfo;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.SendableStringArray;
import frc.util.SlewRateLimiter3d;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import io.github.oblarg.oblog.annotations.Log.ToString;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;
import org.photonvision.PhotonCamera;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;
import org.photonvision.targeting.PhotonTrackedTarget;
public class Vision extends SubsystemBase implements Loggable{
  CameraParams targetCam;
  SlewRateLimiter3d slewLimiter=new SlewRateLimiter3d(0.1, 0.1);
  // CameraParams driveCam;
  public Vision() {
    targetCam = new CameraParams("target");
    // driveCam  = new CameraParams("drive");
  }

  public boolean hasTargets(){
    return targetCam.isConnected && targetCam.camera.getLatestResult().hasTargets();
  }
  public Pose3d getRobotPosFromBestTarget(){
    PhotonTrackedTarget target = targetCam.camera.getLatestResult().getBestTarget();
    int targetId = target.getFiducialId();
    Pose3d targetPos = Constants.VISION.aprilTagPositions.get(targetId);
    Transform3d cameraToTarget=target.getCameraToTarget();
    Pose3d camPos = targetPos.transformBy(cameraToTarget.inverse());
    Pose3d robotPos = camPos.transformBy(Constants.VISION.cameraToRobot);

    //multiplier that should make the returned position more accurate (error is pretty consistent)
    robotPos= slewLimiter.calculate(new Pose3d(new Translation3d(robotPos.getX()*2.6/2.8, robotPos.getY(),robotPos.getZ()),robotPos.getRotation()));
    return robotPos; 
  }
  public double getLatency(){
    SmartDashboard.putNumber("latency", targetCam.camera.getLatestResult().getLatencyMillis());
    return targetCam.camera.getLatestResult().getLatencyMillis();
  }
  @Override
  public void periodic() {
    targetCam.connect();
    // driveCam.connectToCam();
    updateTopTargetPositions();
  }

  /*Telemetry Code-----*/
@Log.BooleanBox(name="Target Cam Connected?", rowIndex=0,columnIndex=15,width=15,height=5)
public boolean isTargetCamConnected(){
  return targetCam.isConnected;
}
// @Log.BooleanBox(name="Drive Cam Connected?", rowIndex=0,columnIndex=0,width=15,height=5)
// public boolean isDriveCamConnected(){
//   return driveCam.isConnected;
// }
@Log.Dial(name="Target Num", rowIndex=0,columnIndex=30,width=15,height=10)
public int getTargetNum(){
  if(hasTargets())
    return 0;
  return targetCam.camera.getLatestResult().targets.size();
}
private List<String> topTargetPositions = new ArrayList<String>(){{add("");add("");add("");}}; 
@Log(name="Target Positions", rowIndex=10,columnIndex=30,width=15,height=6)
private SendableStringArray sendableTopTargetPositions = new SendableStringArray(()->topTargetPositions.get(0),()->topTargetPositions.get(1),()->topTargetPositions.get(2));
private void updateTopTargetPositions(){
  int index=0;
  if(hasTargets()){
    List<Translation3d> translations=targetCam.camera.getLatestResult().targets.stream().map((target)->target.getCameraToTarget().getTranslation()).collect(Collectors.toList());
    while(index<translations.size()){
      topTargetPositions.set(index,translations.get(index).toString().substring(13));
      index++;
    }
  }
  while(index<3){
    topTargetPositions.set(index, "");
    index++;
  }
}
@Log(name="Vision-Measured Robot Pos", rowIndex=16, columnIndex=30,width=15, height=5)
private String getVisionMeasuredRobotPos(){
  if(!hasTargets())
    return "";
  Pose3d robotPos=getRobotPosFromBestTarget();
  robotPos= new Pose3d(new Translation3d(robotPos.getX()*2.6/2.8, robotPos.getY(),robotPos.getZ()),robotPos.getRotation());
  return robotPos.toString().substring(20);
}
}
