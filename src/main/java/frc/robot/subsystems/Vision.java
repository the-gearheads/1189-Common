// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.SendableStringArray;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
public class Vision extends SubsystemBase implements Loggable{
  /** Creates a new Vision. */
  PhotonCamera camera;
  boolean isConnected = false;
  public Vision() {
    isConnected = false;
  }
  
  public boolean connectToPhotonVision(){
    try{
      camera = new PhotonCamera("TargetCamera");
      return true;
    }catch(Exception e){
      DriverStation.reportError("Cannot connect to Photon Vision", false);
      return false;
    }
  }
  @Log.BooleanBox(name="connected?", rowIndex=0,columnIndex=0,width=10,height=10)
  public boolean isConnected(){
    return isConnected;
  }
  @Log.Dial(name="Target Num", rowIndex=0,columnIndex=10,width=10,height=10)
  public int getTargetNum(){
    if(isConnected){
      return camera.getLatestResult().targets.size();
    }else{
      return 0;
    }
  }
  private List<String> topTargetPositions = new ArrayList<String>(){{add("");add("");add("");}}; 
  @Log(name="Target Positions", rowIndex=10,columnIndex=0,width=20,height=20)
  private SendableStringArray sendableTopTargetPositions = new SendableStringArray(()->topTargetPositions.get(0),()->topTargetPositions.get(1),()->topTargetPositions.get(2));
  private void updateTopTargetPositions(){
    int index=0;
    if(isConnected){
      List<Translation3d> translations=camera.getLatestResult().targets.stream().map((target)->target.getCameraToTarget().getTranslation()).collect(Collectors.toList());
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
    PhotonTrackedTarget target = camera.getLatestResult().getBestTarget();
    int targetId = target.getFiducialId();
    Translation2d targetToCamera=target.getCameraToTarget().getTranslation().toTranslation2d().unaryMinus();
    
    Translation2d fieldToTarget = Constants.VISION.aprilTagPositions.get(targetId);
    Translation2d fieldToCamera = fieldToTarget.plus(targetToCamera);
    Translation2d fieldToRobot = fieldToCamera.plus(Constants.VISION.cameraToRobot);
    return fieldToRobot; 
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    isConnected = connectToPhotonVision();
    updateTopTargetPositions();
  }
}
