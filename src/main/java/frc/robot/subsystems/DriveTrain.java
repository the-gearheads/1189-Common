// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Util.Gyroscope;
import frc.Util.Shuffleboard.SBNumber;
import frc.Util.Shuffleboard.SBTab;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.SPI;


public class DriveTrain extends SubsystemBase {

  //Initialize motors
  private final WPI_TalonFX rfMotor = new WPI_TalonFX(Constants.DriveTrain.RFMOTOR_ID);                                     // right-front motor
  private final WPI_TalonFX rbMotor = new WPI_TalonFX(Constants.DriveTrain.RBMOTOR_ID);                                     // right-back motor
  private final WPI_TalonFX lfMotor = new WPI_TalonFX(Constants.DriveTrain.LFMOTOR_ID);                                     // left-front motor
  private final WPI_TalonFX lbMotor = new WPI_TalonFX(Constants.DriveTrain.LBMOTOR_ID);                                     // left-back motor

  private final SimpleMotorFeedforward rightFF = 
            new SimpleMotorFeedforward(Constants.DriveTrain.RIGHT_FF_kS, Constants.DriveTrain.RIGHT_FF_kV);
  private final SimpleMotorFeedforward leftFF = 
            new SimpleMotorFeedforward(Constants.DriveTrain.LEFT_FF_kS, Constants.DriveTrain.LEFT_FF_kV);

  private final DifferentialDriveKinematics kinematics = 
            new DifferentialDriveKinematics(Constants.DriveTrain.TRACK_WIDTH);                                              // Useful in converting controller inputs to wheel speeds
  private DifferentialDriveOdometry odometry = 
      new DifferentialDriveOdometry(new Rotation2d(0));                                                              // Useful in knowing robot position; the initial rotation value will be overridden in the constructor
  private Gyroscope gyro = new Gyroscope(new AHRS(SPI.Port.kMXP), true);                                        // Very useful helper class that can invert the gyroscope (which is used to provide the angle of the robot heading to the odometry object)                                                                   // provides angle to odometry object

  SBTab tab = new SBTab("Drive Subsystem");

  /** Creates a new DriveTrain. */
  public DriveTrain() {

    // Sets direction of motion and integrated sensors
    rfMotor.setInverted(true);
    rbMotor.setInverted(true);
    lfMotor.setInverted(false);
    lbMotor.setInverted(false);

    setPosition(Constants.DriveTrain.START_POSITION);
    populateTab();
  }

  //Creates DriveSystem ShuffleBoard Tab
  private void populateTab(){
    SBNumber gyroAngle = tab.getNumber("gyroNum", 0);
    gyroAngle.setValue(7);
    gyroAngle.setPosition(2,1);
    gyroAngle.setSize(3,3);
    gyroAngle.setPeriodic(()->{
        gyroAngle.setValue(gyroAngle.getValue(0)+ 1);
    });
  } 

  // Encoder-related methods
  public double getRightPos(){
    double meanEncoderVal = (rfMotor.getSelectedSensorPosition() + rbMotor.getSelectedSensorPosition()) / 2; // average of right side native encoder units
    double rotations = meanEncoderVal / Constants.DriveTrain.TALON_UNITS_PER_ROTATION;                       // convert native units to rotations
    double wheelRotations = rotations * Constants.DriveTrain.SHAFT_TO_WHEEL_GEAR_RATIO;                      // convert rotations of motor shaft to rotations of wheel
    double linearDisplacement = wheelRotations * Constants.DriveTrain.WHEEL_CIRCUMFERENCE;                   // convert wheel rotations to linear displacement
    return linearDisplacement;
  }

  public double getLeftPos(){
    double meanEncoderVal = (lfMotor.getSelectedSensorPosition() + lbMotor.getSelectedSensorPosition()) / 2; // average of right side native encoder units
    double rotations = meanEncoderVal / Constants.DriveTrain.TALON_UNITS_PER_ROTATION;                       // convert native units to rotations
    double wheelRotations = rotations * Constants.DriveTrain.SHAFT_TO_WHEEL_GEAR_RATIO;                      // convert rotations of motor shaft to rotations of wheel
    double linearDisplacement = wheelRotations * Constants.DriveTrain.WHEEL_CIRCUMFERENCE;                   // convert wheel rotations to linear displacement
    return linearDisplacement;
  }

  public double getRightVel(){
    double meanEncoderVal = (rfMotor.getSelectedSensorVelocity() + rbMotor.getSelectedSensorVelocity()) / 2; // average of right side native encoder units
    double rotations = meanEncoderVal / Constants.DriveTrain.TALON_UNITS_PER_ROTATION;                       // convert native units to rotations
    double wheelRotations = rotations * Constants.DriveTrain.SHAFT_TO_WHEEL_GEAR_RATIO;                      // convert rotations of motor shaft to rotations of wheel
    double linearDisplacement = wheelRotations * Constants.DriveTrain.WHEEL_CIRCUMFERENCE;                   // convert wheel rotations to linear displacement
    return linearDisplacement;
  }

  public double getLeftVel(){
    double meanEncoderVal = (lfMotor.getSelectedSensorVelocity() + lbMotor.getSelectedSensorVelocity()) / 2; // average of right side native encoder units
    double rotations = meanEncoderVal / Constants.DriveTrain.TALON_UNITS_PER_ROTATION;                       // convert native units to rotations
    double wheelRotations = rotations * Constants.DriveTrain.SHAFT_TO_WHEEL_GEAR_RATIO;                      // convert rotations of motor shaft to rotations of wheel
    double linearDisplacement = wheelRotations * Constants.DriveTrain.WHEEL_CIRCUMFERENCE;                   // convert wheel rotations to linear displacement
    return linearDisplacement;
  }

  // Position-related methods
  public void setPosition(Pose2d pos){
    rfMotor.setSelectedSensorPosition(0);
    rbMotor.setSelectedSensorPosition(0);
    lfMotor.setSelectedSensorPosition(0);
    lbMotor.setSelectedSensorPosition(0);
    gyro.setRotation2d(pos.getRotation());                                                                   // provide gyro with negative angle, since the gyro records counterclockwise motion
    odometry.resetPosition(pos, gyro.getRotation2d());                                                       // provide odometry with negative of gyroscope angle, since gyroscope records counterclockwise motion
  }

  // Gyroscope-related methods
  public double getContinuousGyroAngle(){
    return gyro.getContinuousAngle();
  }

  public void updateOdometry(){
    odometry.update(gyro.getRotation2d(), getLeftPos(), getRightPos());
  }

  // Drive-related methods
  public void setRawSpeeds(DifferentialDriveWheelSpeeds speeds){
    rfMotor.set(speeds.rightMetersPerSecond);
    rbMotor.set(speeds.rightMetersPerSecond);
    lfMotor.set(speeds.leftMetersPerSecond);
    lbMotor.set(speeds.leftMetersPerSecond);
  }

  public DifferentialDriveWheelSpeeds calculateFF(DifferentialDriveWheelSpeeds speeds){
    double rightFFSpeed = rightFF.calculate(speeds.rightMetersPerSecond);
    double leftFFSpeed = leftFF.calculate(speeds.leftMetersPerSecond);

    return new DifferentialDriveWheelSpeeds(leftFFSpeed, rightFFSpeed);
  }

  // PID-related methods
  private void setMotorPID(WPI_TalonFX motor, double kP, double kI, double kD){ // Just makes the setPID() method more readable
    motor.config_kP(0, kP);
    motor.config_kI(0, kI);
    motor.config_kD(0, kD);
  }
  public void setPID(double kP, double kI, double kD){
    setMotorPID(rfMotor, kP, kI, kD);
    setMotorPID(rbMotor, kP, kI, kD);
    setMotorPID(lfMotor, kP, kI, kD);
    setMotorPID(lbMotor, kP, kI, kD);
  }

  // Acceleration-related methods
  public void setRampRate(double rampRate){  // Ramp rate is how much time it takes to reach max motor speed
    rfMotor.configOpenloopRamp(rampRate);
    rbMotor.configOpenloopRamp(rampRate);
    lfMotor.configOpenloopRamp(rampRate);
    lbMotor.configOpenloopRamp(rampRate);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    updateOdometry(); //Must be continuously called to maintain an accurate position of the robot
    tab.periodic();
  }
}
