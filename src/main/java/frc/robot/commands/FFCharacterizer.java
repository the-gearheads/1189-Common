// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.util.Shuffleboard.SBBoolean;
import frc.util.Shuffleboard.SBColor;
import frc.util.Shuffleboard.SBGroup;
import frc.util.Shuffleboard.SBNumber;
import frc.util.Shuffleboard.SBNumberGroup;
import frc.util.Shuffleboard.SBString;
import frc.util.Shuffleboard.SBTab;

public class FFCharacterizer extends CommandBase {
  private DriveTrain driveTrain;
  private SBTab tab;
  private boolean isRunning;
  private int nextVoltageVal;
  private Timer timer;
  private Map<String, Double> speedData;
  HashMap<Double, Map<String, Double>> voltageSpeedsMap = new HashMap<Double, Map<String, Double>>();

  /** Creates a new FFCharacterizer. */
  public FFCharacterizer(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    this.tab = new SBTab("FF Characterizer");
    isRunning = false;
    nextVoltageVal = 1;
    resetSpeedData();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // private void populateTab(){
  //   SBBoolean startCharacterizer = tab.getBoolean("Start", false)
  //   .setView(BuiltInWidgets.kToggleButton)
  //   .setSize(1,1)
  //   .setPosition(40,0)
  //   .setPeriodic((current)->{
  //     isRunning = current;
  //   });

  //   SBNumberGroup Angle = tab.getNumberGroup("Left Speed", 0);
  //   leftSpeed.setPosition(0,0);
  //   leftSpeed.setWidth(10);
  //   leftSpeed.setPeriodic(()->getLeftVel());

  //   SBNumberGroup reqLeftSpeed = tab.getNumberGroup("req Left Speed", 0);
  //   reqLeftSpeed.setPosition(10,0);
  //   reqLeftSpeed.setWidth(10);
  //   reqLeftSpeed.setPeriodic(()->getLeftVel());

  //   SBNumberGroup rightSpeed = tab.getNumberGroup("Right Speed", 0);
  //   rightSpeed.setPosition(20,0);
  //   rightSpeed.setWidth(10);
  //   rightSpeed.setPeriodic(()->getLeftVel());

  //   SBNumberGroup reqRightSpeed = tab.getNumberGroup("req Right Speed", 0);
  //   reqRightSpeed.setPosition(30,0);
  //   reqRightSpeed.setWidth(10);
  //   reqRightSpeed.setPeriodic(()->getLeftVel());
  // }

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // pausing capabilities
    if(!isRunning){
      driveTrain.setRawSpeeds(new DifferentialDriveWheelSpeeds(0, 0));
      timer.stop();
      return;
    }
    timer.start();
    
    driveTrain.setRawSpeeds(new DifferentialDriveWheelSpeeds(voltageSpeedsMap.size()+1, voltageSpeedsMap.size()+1));
    if(timer.get() % 3 > 1){
      updateSpeedData();
    }

    if(timer.get() / 3 > voltageSpeedsMap.size() + 1){
      Map<String, Double> avgSpeeds = Map.of("left", speedData.get("total left") / speedData.get("left sample count"),
                                           "right", speedData.get("total right") / speedData.get("right sample count"));
      voltageSpeedsMap.put(voltageSpeedsMap.size() + 1.0, avgSpeeds);

      resetSpeedData();
    }

    tab.periodic();
  }

  private void updateSpeedData(){
    speedData.replace("left total", speedData.get("left total")+driveTrain.getLeftVel());
    speedData.replace("right total", speedData.get("right total")+driveTrain.getRightVel());
  }
  private void resetSpeedData(){
    speedData = Map.of("total left", 0.0, "total right", 0.0, "left sample count", 0.0, "right sample count", 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setRawSpeeds(new DifferentialDriveWheelSpeeds(0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return voltageSpeedsMap.size() >= 12;
  }
}
