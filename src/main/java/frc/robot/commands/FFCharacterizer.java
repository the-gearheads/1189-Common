// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.Util.Shuffleboard.SBBoolean;
import frc.Util.Shuffleboard.SBNumber;
import frc.Util.Shuffleboard.SBTab;
import frc.robot.subsystems.DriveTrain;

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
    populateTab();
  }

  private void populateTab(){
    SBBoolean startCharacterizer = tab.getBoolean("Start", false)
    .setView(BuiltInWidgets.kToggleButton)
    .setSize(1,1)
    .setPosition(11,0)
    .setPeriodic((current)->{
      isRunning = current;
    });

    SBNumber leftSpeedGraph = tab.getNumber("Left Speed Graph", 0)
    .setView(BuiltInWidgets.kGraph)
    .setSize(3,3)
    .setPosition(0,0)
    .setPeriodic(()->{
      return driveTrain.getLeftVel();
    });

    SBNumber leftSpeedVal = tab.getNumber("Left Speed Val", 0)
    .setSize(3,1)
    .setPosition(3,0)
    .setPeriodic(()->{
      return driveTrain.getLeftVel();
    });

    SBNumber reqLeftSpeedGraph = tab.getNumber("Requested Left Speed Graph", 0)
    .setView(BuiltInWidgets.kGraph)
    .setSize(3,3)
    .setPosition(0,0)
    .setPeriodic(()->{
      return driveTrain.getLeftVel();
    });

    SBNumber reqLeftSpeedVal = tab.getNumber("Requested Left Speed Val", 0)
    .setSize(3,1)
    .setPosition(3,0)
    .setPeriodic(()->{
      return driveTrain.getLeftVel();
    });

    SBNumber rightSpeedGraph = tab.getNumber("Left Speed Graph", 0)
    .setView(BuiltInWidgets.kGraph)
    .setSize(3,3)
    .setPosition(15,0)
    .setPeriodic(()->{
      return driveTrain.getLeftVel();
    });

    SBNumber rightSpeedVal = tab.getNumber("Left Speed Val", 0)
    .setSize(3,1)
    .setPosition(15,3)
    .setPeriodic(()->{
      return driveTrain.getLeftVel();
    });

    SBNumber reqRightSpeedGraph = tab.getNumber("Requested Left Speed Graph", 0)
    .setView(BuiltInWidgets.kGraph)
    .setSize(3,3)
    .setPosition(18,0)
    .setPeriodic(()->{
      return driveTrain.getLeftVel();
    });

    SBNumber reqRightSpeedVal = tab.getNumber("Requested Left Speed Val", 0)
    .setSize(3,1)
    .setPosition(18,3)
    .setPeriodic(()->{
      return driveTrain.getLeftVel();
    });
  }

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
