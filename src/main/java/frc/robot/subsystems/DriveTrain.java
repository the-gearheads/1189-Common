// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// An ode to thyself
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.driveTrain.ArcadeDrive;
import frc.robot.controllers.Controllers;
import frc.util.Gyroscope;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;


public class DriveTrain extends SubsystemBase  implements Loggable{

  //Initialize motors
  private final WPI_TalonFX rfMotor = new WPI_TalonFX(Constants.DRIVE.RFMOTOR_ID);  // right-front motor
  private final WPI_TalonFX rbMotor = new WPI_TalonFX(Constants.DRIVE.RBMOTOR_ID);  // right-back motor
  private final WPI_TalonFX lfMotor = new WPI_TalonFX(Constants.DRIVE.LFMOTOR_ID);  // left-front motor
  private final WPI_TalonFX lbMotor = new WPI_TalonFX(Constants.DRIVE.LBMOTOR_ID);  // left-back motor
  private final SimpleMotorFeedforward leftFeedForward = 
    new SimpleMotorFeedforward(Constants.DRIVE.LEFT_FF_kS,
                               Constants.DRIVE.LEFT_FF_kV,
                               Constants.DRIVE.LEFT_FF_kA);
  private final SimpleMotorFeedforward rightFeedForward = 
    new SimpleMotorFeedforward(Constants.DRIVE.RIGHT_FF_kS,
                               Constants.DRIVE.RIGHT_FF_kV,
                               Constants.DRIVE.RIGHT_FF_kA);

  private final DifferentialDriveKinematics kinematics = 
    new DifferentialDriveKinematics(Constants.DRIVE.TRACK_WIDTH);  // Useful in converting controller inputs to wheel speeds
  private DifferentialDrivePoseEstimator odometry = 
    new DifferentialDrivePoseEstimator(new Rotation2d(0),
                                      Constants.DRIVE.ZERO_POSITION, 
                                      Constants.DRIVE.STATE_SD,    // State measurement standard deviations. X, Y, theta.
                                      Constants.DRIVE.LOCAL_SD,    // Local measurement standard deviations. Left encoder, right encoder, gyro.
                                      Constants.DRIVE.GLOBAL_SD);  // Global measurement standard deviations. X, Y, and theta.)
    @Log.Gyro(columnIndex=20, rowIndex=0, width=10, height=10)
  private Gyroscope gyro = 
    new Gyroscope(SPI.Port.kMXP, true);      // Very useful helper class that can invert the gyroscope (which is used to provide the angle of the robot heading to the odometry object)                                                                   // provides angle to odometry object

  // Telemetry 
  @Log(name="Field", rowIndex=30,columnIndex = 0,width=18,height=11)
  Field2d field = new Field2d();
  @Log.DifferentialDrive(tabName= "DriveTrain: Config", name="Motor Controller", rowIndex=10, columnIndex = 20, width=15, height=10)
  DifferentialDrive differentialDrive = 
    new DifferentialDrive(new MotorControllerGroup(lfMotor,lbMotor), new MotorControllerGroup(rfMotor,rbMotor));

  //Initialize all Simulation-Related Fields
  private TalonFXSimCollection rSim = rfMotor.getSimCollection();
  private TalonFXSimCollection lSim = lfMotor.getSimCollection();

  private SimDouble simAngle;
  private DifferentialDrivetrainSim sim;

  /** Creates a new DriveTrain. */
  public DriveTrain() {

    rfMotor.configFactoryDefault();
    rbMotor.configFactoryDefault();
    lfMotor.configFactoryDefault();
    lbMotor.configFactoryDefault();

    //rear motors follow front motors
    rbMotor.follow(rfMotor);
    lbMotor.follow(lfMotor);

    // Sets direction of motion and integrated sensors
    rfMotor.setInverted(true);
    rbMotor.setInverted(InvertType.FollowMaster);
    lfMotor.setInverted(false);
    lbMotor.setInverted(InvertType.FollowMaster);

    // Set up gyro simulation 
    int dev      = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        simAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    
    setPose(Constants.DRIVE.START_POSITION);
    setDefaultCommand(new ArcadeDrive(this, ()->Controllers.driverController.getMoveAxis(), ()->Controllers.driverController.getRotateAxis()));
  }

  // Encoder-related methods
  @Log.Graph(columnIndex=0, rowIndex=0, width=10, height=10)
  public double getLeftPos(){
    double meanEncoderVal     = (lfMotor.getSelectedSensorPosition() + lbMotor.getSelectedSensorPosition()) / 2;  // average of right side native encoder units
    double rotations          = meanEncoderVal / Constants.DRIVE.TALON_UNITS_PER_ROTATION;                        // convert native units to rotations
    double wheelRotations     = rotations / Constants.DRIVE.SHAFT_TO_WHEEL_GEAR_RATIO;                            // convert rotations of motor shaft to rotations of wheel
    double linearDisplacement = wheelRotations * Constants.DRIVE.WHEEL_CIRCUMFERENCE;                             // convert wheel rotations to linear displacement
    return linearDisplacement;
  }
  @Log.Graph(columnIndex=10, rowIndex=0, width=10, height=10)
  public double getRightPos(){
    double meanEncoderVal     = (rfMotor.getSelectedSensorPosition() + rbMotor.getSelectedSensorPosition()) / 2;  // average of right side native encoder units
    double rotations          = meanEncoderVal / Constants.DRIVE.TALON_UNITS_PER_ROTATION;                        // convert native units to rotations
    double wheelRotations     = rotations / Constants.DRIVE.SHAFT_TO_WHEEL_GEAR_RATIO;                            // convert rotations of motor shaft to rotations of wheel
    double linearDisplacement = wheelRotations * Constants.DRIVE.WHEEL_CIRCUMFERENCE;                             // convert wheel rotations to linear displacement
    return linearDisplacement;
  }
  @Log.Graph(columnIndex=0, rowIndex=10, width=10, height=10)
  public double getLeftVel(){
    double meanEncoderVal      = (lfMotor.getSelectedSensorVelocity() + lbMotor.getSelectedSensorVelocity()) / 2;  // average of right side native encoder units
           meanEncoderVal     *= 10;                                                                               //Convert (Native Units/100ms) -> (Native Units/1s)
    double rotations           = meanEncoderVal / Constants.DRIVE.TALON_UNITS_PER_ROTATION;                        // convert native units to rotations
    double wheelRotations      = rotations / Constants.DRIVE.SHAFT_TO_WHEEL_GEAR_RATIO;                            // convert rotations of motor shaft to rotations of wheel
    double linearDisplacement  = wheelRotations * Constants.DRIVE.WHEEL_CIRCUMFERENCE;                             // convert wheel rotations to linear displacement
    return linearDisplacement;
  }
  @Log.Graph(columnIndex=10, rowIndex=10, width=10, height=10)
  public double getRightVel(){
    double meanEncoderVal      = (rfMotor.getSelectedSensorVelocity() + rbMotor.getSelectedSensorVelocity()) / 2;  // average of right side native encoder units
           meanEncoderVal     *= 10;                                                                               //Convert (Native Units/100ms) -> (Native Units/1s)
    double rotations           = meanEncoderVal / Constants.DRIVE.TALON_UNITS_PER_ROTATION;                        // convert native units to rotations
    double wheelRotations      = rotations / Constants.DRIVE.SHAFT_TO_WHEEL_GEAR_RATIO;                            // convert rotations of motor shaft to rotations of wheel
    double linearDisplacement  = wheelRotations * Constants.DRIVE.WHEEL_CIRCUMFERENCE;                             // convert wheel rotations to linear displacement
    return linearDisplacement;
  }

  // Odometry-related methods
  public void setPose(Pose2d pos){
    rfMotor.setSelectedSensorPosition(0);
    rbMotor.setSelectedSensorPosition(0);
    lfMotor.setSelectedSensorPosition(0);
    lbMotor.setSelectedSensorPosition(0);
    gyro.setRotation2d(pos.getRotation());                                                                   // provide gyro with negative angle, since the gyro records counterclockwise motion
    odometry.resetPosition(pos, gyro.getRotation2d());                                                       // provide odometry with negative of gyroscope angle, since gyroscope records counterclockwise motion

    var drivetrain = LinearSystemId.identifyDrivetrainSystem(Constants.DRIVE.SIM.LINEAR_KV,
                                                             Constants.DRIVE.SIM.LINEAR_KA,
                                                             Constants.DRIVE.SIM.ANGULAR_KV,
                                                             Constants.DRIVE.SIM.ANGULAR_KA);
        sim        = new DifferentialDrivetrainSim(drivetrain,
                                                   DCMotor.getFalcon500(2),
                                                   Constants.DRIVE.SHAFT_TO_WHEEL_GEAR_RATIO,
                                                   Constants.DRIVE.TRACK_WIDTH,
                                                   Constants.DRIVE.WHEEL_RADIUS, null);
  }
  public Pose2d getPose(){
    return odometry.getEstimatedPosition();
  }
  @Log(name="Pose", rowIndex=12,columnIndex=30,width=18,height=5)
  public String getSendablePose(){
    Pose2d pose = odometry.getEstimatedPosition();
    return "["+
            ((double) Math.round(pose.getX() * 100.0) / 100.0)+", "+
            ((double) Math.round(pose.getY() * 100.0) / 100.0)+", "+
            ((double) Math.round(pose.getRotation().getDegrees() * 100.0) / 100.0)+"]";
  }
  public void updateOdometry(){
    var wheelSpeeds = new DifferentialDriveWheelSpeeds(getLeftVel(), getRightVel());
    odometry.update(gyro.getRotation2d(), wheelSpeeds, getLeftPos(), getRightPos());
  }
  public void addVisionMeasurement(Pose2d pose){
    odometry.addVisionMeasurement(pose, Timer.getFPGATimestamp());
  }
  @Config(rowIndex=0, columnIndex=10, width=10, height=20)
  private void setOdometry(@Config.NumberSlider double s0, @Config.NumberSlider double s1, @Config.NumberSlider double s2, @Config.NumberSlider double s3, @Config.NumberSlider double s4,
                           @Config.NumberSlider double l0, @Config.NumberSlider double l1, @Config.NumberSlider double l2,
                           @Config.NumberSlider double g0, @Config.NumberSlider double g1, @Config.NumberSlider double g2){

  }

  // Gyroscope-related methods
  public double getContinuousGyroAngle(){
    return gyro.getContinuousAngle();
  }

  

  // Kinematics-related methods
  public DifferentialDriveWheelSpeeds toWheelSpeeds(ChassisSpeeds chassisSpeeds){
    return kinematics.toWheelSpeeds(chassisSpeeds);
  }
  public DifferentialDriveWheelSpeeds calculateFF(DifferentialDriveWheelSpeeds speeds){
    double rightFFSpeed = rightFeedForward.calculate(speeds.rightMetersPerSecond);
    double leftFFSpeed  = leftFeedForward.calculate(speeds.leftMetersPerSecond);

    return new DifferentialDriveWheelSpeeds(leftFFSpeed, rightFFSpeed);
  }

  // Drive-related methods
  public void setRawSpeeds(DifferentialDriveWheelSpeeds speeds){
    speeds.rightMetersPerSecond = MathUtil.clamp(speeds.rightMetersPerSecond, -12,12);
    speeds.leftMetersPerSecond  = MathUtil.clamp(speeds.leftMetersPerSecond, -12,12);

    rfMotor.setVoltage(speeds.rightMetersPerSecond);
    rbMotor.setVoltage(speeds.rightMetersPerSecond);
    lfMotor.setVoltage(speeds.leftMetersPerSecond);
    lbMotor.setVoltage(speeds.leftMetersPerSecond);
  }
  public void setRawSpeeds(double right, double left){
    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(left, right);
    setRawSpeeds(wheelSpeeds);
  }

  // Motor Config-related methods
  private void setMotorPID(WPI_TalonFX motor, double kP, double kI, double kD, double allowableError){ // Just makes the setPID() method more readable
    motor.config_kP(0, kP);
    motor.config_kI(0, kI);
    motor.config_kD(0, kD);
    motor.configAllowableClosedloopError(0, allowableError, 0);

  }
  @Config(rowIndex=0, columnIndex=0, width=10, height=20)
  public void setPID(@Config.NumberSlider(name="kP") double kP,
                     @Config.NumberSlider(name="kI") double kI,
                     @Config.NumberSlider(name="kD") double kD,
                     @Config.NumberSlider(name="Allowable Error") double allowableError){
    setMotorPID(rfMotor, kP, kI, kD, allowableError);
    setMotorPID(lfMotor, kP, kI, kD, allowableError);
  }
  @Config.ToggleButton(name="brake Mode?", defaultValue = true, rowIndex=0, columnIndex=20, width=15, height=5)
  public void setIdleMode(boolean isBrake){
    if(isBrake){
      rfMotor.setNeutralMode(NeutralMode.Brake);
      lfMotor.setNeutralMode(NeutralMode.Brake);
      rbMotor.setNeutralMode(NeutralMode.Brake);
      lbMotor.setNeutralMode(NeutralMode.Brake);
    }else{
      rfMotor.setNeutralMode(NeutralMode.Coast);
      lfMotor.setNeutralMode(NeutralMode.Coast);
      rbMotor.setNeutralMode(NeutralMode.Coast);
      lbMotor.setNeutralMode(NeutralMode.Coast);
    }
  }
  @Config.NumberSlider(name="RampRate",defaultValue=0, rowIndex=5, columnIndex=20, width=15, height=5)
  public void setRampRate(double rampRate){  // Ramp rate is how much time it takes to reach max motor speed
    rfMotor.configOpenloopRamp(rampRate);
    lfMotor.configOpenloopRamp(rampRate);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    updateOdometry(); //Must be continuously called to maintain an accurate position of the robot
    field.setRobotPose(odometry.getEstimatedPosition());
  }

  // Sim-related Methods
  private int distanceToNativeUnits(double position) { // Necessary Conversions for simulation
    double wheelRotations = position / Constants.DRIVE.WHEEL_CIRCUMFERENCE;
    double motorRotations = wheelRotations * Constants.DRIVE.SHAFT_TO_WHEEL_GEAR_RATIO;
    double nativeUnits    = motorRotations * Constants.DRIVE.TALON_UNITS_PER_ROTATION;
    return (int)(nativeUnits);
  }
  private int velocityToNativeUnits(double velocity) { // Necessary Conversions for simulation
      // i think this should work
      return (int)(distanceToNativeUnits(velocity) / 10);
  }
  @Override
    public void simulationPeriodic() {

        /* Pass the robot battery voltage to the simulated Talon FXs */
        lSim.setBusVoltage(RobotController.getBatteryVoltage());
        rSim.setBusVoltage(RobotController.getBatteryVoltage());

        sim.setInputs(lSim.getMotorOutputLeadVoltage(),
            -rSim.getMotorOutputLeadVoltage());

        /* Advance the model by 20ms */
        sim.update(0.02);

        /* Set odometry for position and velocity based on simulation. */
        lSim.setIntegratedSensorRawPosition(
            distanceToNativeUnits(
                sim.getLeftPositionMeters()
        ));
        lSim.setIntegratedSensorVelocity(
            velocityToNativeUnits(
                sim.getLeftVelocityMetersPerSecond()
        ));
        rSim.setIntegratedSensorRawPosition(
            distanceToNativeUnits(
                -sim.getRightPositionMeters()
        ));
        rSim.setIntegratedSensorVelocity(
            velocityToNativeUnits(
                -sim.getRightVelocityMetersPerSecond()
        ));

        simAngle.set(-sim.getHeading().getDegrees());

    }
}
