// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.utilwhatev.Shuffleboard.SBBoolean;
import frc.utilwhatev.Shuffleboard.SBNumber;
import frc.utilwhatev.Shuffleboard.SBTab;

public class FFCharacterizer extends CommandBase {
  private DriveTrain driveTrain;
  private SBTab tab;
  private boolean isRunning;

  /** Creates a new FFCharacterizer. */
  public FFCharacterizer(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    this.tab = new SBTab("FF Characterizer");
    isRunning = false;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // populateTab();
  }

  // private void populateTab(){
  //   SBBoolean startCharacterizer = tab.getBoolean("Start", false)
  //   .setView(BuiltInWidgets.kToggleButton)
  //   .setSize(1,1)
  //   .setPosition(22,0)
  //   .setPeriodic((current)->{
  //     isRunning = current;
  //   });

  //   SBGroup leftSpeedGroup = tab.getGroup("Left Speed")
  //   .setPosition(0,0)
  //   .setWidth(10);

  //   SBNumber leftSpeedGraph = tab.getNumber("Left Speed Graph", 0)
  //   .setView(BuiltInWidgets.kGraph)
  //   .setSize(10,10)
  //   // .setPosition(0,0)
  //   .setPeriodic(()->{
  //     return getLeftVel();
  //   });

  //   SBNumber leftSpeedVal = tab.getNumber("Left Speed Val", 0)
  //   .setSize(5,1)
  //   // .setPosition(0,10)
  //   .setPeriodic(()->{
  //     return getLeftVel();
  //   });
  //   leftSpeedGroup.append(leftSpeedVal);

  //   SBGroup reqLeftSpeedGroup = tab.getGroup("Left Speed")
  //   .setPosition(10,0)
  //   .setWidth(10);

  //   SBNumber reqLeftSpeedGraph = tab.getNumber("Requested Left Speed Graph", 0)
  //   .setView(BuiltInWidgets.kGraph)
  //   .setSize(10,10)
  //   // .setPosition(10,0)
  //   .setPeriodic(()->{
  //     return getLeftVel();
  //   });
  //   reqLeftSpeedGroup.append(reqLeftSpeedGraph);


  //   SBNumber reqLeftSpeedVal = tab.getNumber("Requested Left Speed Val", 0)
  //   .setSize(10,1)
  //   // .setPosition(10,10)
  //   .setPeriodic(()->{
  //     return getLeftVel();
  //   });
  //   reqLeftSpeedGroup.append(reqLeftSpeedVal);


  //   SBGroup rightSpeedGroup = tab.getGroup("Left Speed")
  //   .setPosition(10,0)
  //   .setWidth(10);

  //   SBNumber rightSpeedGraph = tab.getNumber("Right Speed Graph", 0)
  //   .setView(BuiltInWidgets.kGraph)
  //   // .setPosition(25,0)
  //   .setPeriodic(()->{
  //     return getLeftVel();
  //   });
  //   rightSpeedGroup.append(rightSpeedGraph);

  //   SBNumber rightSpeedVal = tab.getNumber("Right Speed Val", 0)
  //   .setPosition(25,10)
  //   .setSize(10,1)
  //   .setPeriodic(()->{
  //     return getLeftVel();
  //   });

  //   SBNumber reqRightSpeedGraph = tab.getNumber("Requested Right Speed Graph", 0)
  //   .setView(BuiltInWidgets.kGraph)
  //   .setPosition(36,0)
  //   .setPeriodic(()->{
  //     return getLeftVel();
  //   });

  //   SBNumber reqRightSpeedVal = tab.getNumber("Requested Right Speed Val", 0)
  //   .setSize(10,1)
  //   .setPosition(36,10)
  //   .setPeriodic(()->{
  //     return getLeftVel();
  //   });
  // }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tab.periodic();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
