// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.SendableStringArray;
import frc.util.Shuffleboard.SBNumber;
import frc.util.Shuffleboard.SBNumberGroup;
import frc.util.Shuffleboard.SBTab;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import io.github.oblarg.oblog.annotations.Log.ToString;

import java.io.IOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.SocketAddress;
import java.net.SocketTimeoutException;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Function;
import java.util.stream.Collectors;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
public class Vision extends SubsystemBase implements Loggable{
  /** Creates a new Vision. */
  PhotonCamera camera;
  boolean isConnected = false;
  private double lastTime = 0;
  @Log(name="Host", rowIndex=7,columnIndex=0,width=10,height=3)
  String host = "None";
  Timer timer = new Timer();
  public Vision() {
    isConnected = false;
  }
  
  public boolean connectToPhotonVision(){
    Socket  s          = null;
    String  reason     = null ;
    int     exitStatus = 0;
    boolean connected  = false;
    host = "none";
    try {
        s = new Socket();
        s.setReuseAddress(true);
        SocketAddress sa = new InetSocketAddress("localhost", 5800);
        s.connect(sa, 2 * 1000);
    } catch (IOException e) {
        if ( e.getMessage().equals("Connection refused")) {
            reason = "port " + 5800 + " on " + "localhost" + " is closed.";
        };
        if ( e instanceof UnknownHostException ) {
            reason = "node " + "localhost" + " is unresolved.";
        }
        if ( e instanceof SocketTimeoutException ) {
            reason    = "timeout while attempting to reach node " + "localhost" + " on port " + 5800;
        }
    } finally {
        if (s != null) {
            if ( s.isConnected()) {
                System.out.println("Port " + 5800 + " on " + "localhost" + " is reachable!");
                connected = true;
                host="localhost:5800";
                exitStatus = 0;
            } else {
                System.out.println("Port " + 5800 + " on " + "localhost" + " is not reachable; reason: " + reason );
            }
            try {
                s.close();
            } catch (IOException e) {
            }
        }
    }
    if(connected){
      camera = new PhotonCamera("TargetCamera");
    }
      return connected;
  }
  @Log.BooleanBox(name="connected?", rowIndex=0,columnIndex=0,width=10,height=7)
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
    if(isConnected && camera.getLatestResult().hasTargets()){
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
    if(timer.get() - lastTime > 10){
      lastTime = timer.get();
      isConnected = connectToPhotonVision();
      updateTopTargetPositions();
    }
  }
}
