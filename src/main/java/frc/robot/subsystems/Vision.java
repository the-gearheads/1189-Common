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
  int averageSample = 20;
  double slewRate = 0.03;
  Function<Double, Double> area2Dist = (area)->{
    double c1 = 97.69, c2 = -76.9, c3 = 1;
    return c1 + c2 * Math.pow(area,c3);
  };
  SBTab tab;
  public Vision() {
    camera = new PhotonCamera("Camera");
    slewFilter = new SlewRateLimiter(slewRate);
    averageFilter = LinearFilter.movingAverage(200);
    
    tab = new SBTab("Vision");
    populateTab();
  }
  public void populateTab(){
    SBNumberGroup avgArea = tab.getNumberGroup("Average Target Area", 0)
    .setPosition(0,0)
    .setWidth(10)
    .setPeriodic(()->{
      PhotonPipelineResult result = camera.getLatestResult();
      if(result.hasTargets()){
        return averageFilter.calculate(slewFilter.calculate(result.getBestTarget().getArea()));
      }else{
        return 0.0;
      }
    });
    SBNumberGroup area = tab.getNumberGroup("Target Area", 0)
    .setPosition(10,0)
    .setWidth(10)
    .setPeriodic(()->{
      PhotonPipelineResult result = camera.getLatestResult();
      if(result.hasTargets()){
        return result.getBestTarget().getArea();
      }else{
        return 0.0;
      }
    });

    SBNumberGroup avgDist = tab.getNumberGroup("Average Target Distance", 0)
    .setPosition(20,0)
    .setWidth(10)
    .setPeriodic(()->{
      PhotonPipelineResult result = camera.getLatestResult();
      if(result.hasTargets()){
        return area2Dist.apply(averageFilter.calculate(slewFilter.calculate(result.getBestTarget().getArea())));
      }else{
        return 0.0;
      }
    });

    SBNumber averageSlider = tab.getNumber("Average Sample Slider", 0)
    .setPosition(0, avgArea.getSize().height)
    .setView(BuiltInWidgets.kTextView)
    .setSize(10,5)
    .setPeriodic((value)->{
      if(value != averageSample && value > 1){
        averageSample = (int) Math.round(value);
        averageFilter = LinearFilter.movingAverage(averageSample);
      }
    });
    SBNumber medianSlider = tab.getNumber("SlewRate Slider", 0)
    .setPosition(10, area.getSize().height)
    .setView(BuiltInWidgets.kTextView)
    .setSize(10,5)
    .setPeriodic((value)->{
      if(value != slewRate && value > 0){
        slewRate = value;
        slewFilter = new SlewRateLimiter(slewRate);
      }
    });
  }

  @Override
  public void periodic() {
    tab.periodic();
    // This method will be called once per scheduler run
  }
}
