// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.ConnectionInfo;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.SendableStringArray;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import io.github.oblarg.oblog.annotations.Log.ToString;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
public class Vision extends SubsystemBase implements Loggable{
  /** Creates a new Vision. */
  CameraParams targetCam;
  CameraParams driveCam;
  public Vision() {
    targetCam = new CameraParams("target");
    driveCam  = new CameraParams("drive");
  }
  
  public void connectToCam(CameraParams camParams){
      camParams.camera = new PhotonCamera(camParams.name);
      if(camParams.lastLatencyVal == camParams.camera.getLatestResult().getLatencyMillis())
        camParams.equivalencyCount++;
      else
        camParams.equivalencyCount=0;
      if(camParams.equivalencyCount>10)
        camParams.isConnected = false;
      else 
        camParams.isConnected = true;

  }
  @Log.BooleanBox(name="Target Cam Connected?", rowIndex=0,columnIndex=15,width=15,height=5)
  public boolean isTargetCamConnected(){
    return targetCam.isConnected;
  }
  @Log.BooleanBox(name="Drive Cam Connected?", rowIndex=0,columnIndex=0,width=15,height=5)
  public boolean isDriveCamConnected(){
    return driveCam.isConnected;
  }
  @Log.Dial(name="Target Num", rowIndex=0,columnIndex=30,width=15,height=10)
  public int getTargetNum(){
    if(targetCam.isConnected){
      return targetCam.camera.getLatestResult().targets.size();
    }else{
      return 0;
    }
  }
  private List<String> topTargetPositions = new ArrayList<String>(){{add("");add("");add("");}}; 
  @Log(name="Target Positions", rowIndex=10,columnIndex=30,width=15,height=20)
  private SendableStringArray sendableTopTargetPositions = new SendableStringArray(()->topTargetPositions.get(0),()->topTargetPositions.get(1),()->topTargetPositions.get(2));
  private void updateTopTargetPositions(){
    int index=0;
    if(targetCam.isConnected && targetCam.camera.getLatestResult().hasTargets()){
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

  public Translation2d getRobotPosFromBestTarget(){
    PhotonTrackedTarget target = targetCam.camera.getLatestResult().getBestTarget();
    int targetId = target.getFiducialId();
    Translation2d targetToCamera=target.getCameraToTarget().getTranslation().toTranslation2d().unaryMinus();
    
    Translation2d fieldToTarget = Constants.VISION.aprilTagPositions.get(targetId);
    Translation2d fieldToCamera = fieldToTarget.plus(targetToCamera);
    Translation2d fieldToRobot = fieldToCamera.plus(Constants.VISION.cameraToRobot);
    return fieldToRobot; 
  }


  @Override
  public void periodic() {
    connectToCam(targetCam);
    connectToCam(driveCam);
    updateTopTargetPositions();
  }
}
