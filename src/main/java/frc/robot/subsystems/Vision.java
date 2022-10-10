// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.Shuffleboard.SBNumber;
import frc.util.Shuffleboard.SBNumberGroup;
import frc.util.Shuffleboard.SBTab;

import java.util.function.Function;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  PhotonCamera camera;
  SlewRateLimiter slewFilter;
  LinearFilter averageFilter;
  int averageSampleNum = 20;
  double slewRate = 0.03;
  Function<Double, Double> area2Dist = (area)->{
    double c1 = 97.69, c2 = -76.9, c3 = 1;
    return c1 + c2 * Math.pow(area,c3);
  };
  public Vision() {
    camera = new PhotonCamera("Camera");
    slewFilter = new SlewRateLimiter(slewRate);
    averageFilter = LinearFilter.movingAverage(averageSampleNum);
  }

  private void setSlewRate(){
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
