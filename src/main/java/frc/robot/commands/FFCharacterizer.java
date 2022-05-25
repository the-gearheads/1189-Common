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
import frc.utilwhatev.Shuffleboard.SBBoolean;
import frc.utilwhatev.Shuffleboard.SBGroup;
import frc.utilwhatev.Shuffleboard.SBNumber;
import frc.utilwhatev.Shuffleboard.SBString;
import frc.utilwhatev.Shuffleboard.SBTab;

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
    populateTab();
  }

  private void populateTab(){
    SBBoolean pause = tab.getBoolean("Pause", true).setPeriodic((current)->{isRunning = current;})
    .setSize(7,5)
    .setPosition(0,0);

    SBGroup valGroup = tab.getGroup("Values");
    SBNumber voltage = tab.getNumber("Voltage", 0).setPeriodic(()->(voltageSpeedsMap.size()+1.0)).appendTo(valGroup);
    SBNumber leftSpeed = tab.getNumber("Left Speed", 0).setPeriodic(()->driveTrain.getLeftVel()).appendTo(valGroup);
    SBNumber rightSpeed = tab.getNumber("Right Speed", 0).setPeriodic(()->driveTrain.getRightVel()).appendTo(valGroup);
    valGroup.setPosition(15,0);

    SBString output = tab.getString("Output", "").setPeriodic(()->voltageSpeedsMap.toString())
    .setSize(19, 5)
    .setPosition(0,15);
  }


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
