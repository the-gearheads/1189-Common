// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// An ode to thyself
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.driveTrain.ArcadeDrive;
import frc.robot.commands.driveTrain.FFCharacterizer;
import frc.robot.controllers.Controllers;
import frc.util.Gyroscope;
import frc.util.Shuffleboard.SBBoolean;
import frc.util.Shuffleboard.SBGroup;
import frc.util.Shuffleboard.SBNumber;
import frc.util.Shuffleboard.SBNumberGroup;
import frc.util.Shuffleboard.SBTab;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;


public class DriveTrain extends SubsystemBase {

  //Initialize motors
  private final WPI_TalonFX rfMotor = new WPI_TalonFX(Constants.Drive.RFMOTOR_ID);                                     // right-front motor
  private final WPI_TalonFX rbMotor = new WPI_TalonFX(Constants.Drive.RBMOTOR_ID);                                     // right-back motor
  private final WPI_TalonFX lfMotor = new WPI_TalonFX(Constants.Drive.LFMOTOR_ID);                                     // left-front motor
  private final WPI_TalonFX lbMotor = new WPI_TalonFX(Constants.Drive.LBMOTOR_ID);                                     // left-back motor

  private final SimpleMotorFeedforward leftFeedForward = 
            new SimpleMotorFeedforward(Constants.Drive.LEFT_FF_kS, Constants.Drive.LEFT_FF_kV, Constants.Drive.LEFT_FF_kA);
  private final SimpleMotorFeedforward rightFeedForward = 
            new SimpleMotorFeedforward(Constants.Drive.RIGHT_FF_kS, Constants.Drive.RIGHT_FF_kV);

  private final DifferentialDriveKinematics kinematics = 
            new DifferentialDriveKinematics(Constants.Drive.TRACK_WIDTH);                                              // Useful in converting controller inputs to wheel speeds
  private DifferentialDriveOdometry odometry = 
      new DifferentialDriveOdometry(new Rotation2d(0));                                                              // Useful in knowing robot position; the initial rotation value will be overridden in the constructor
  private Gyroscope gyro = new Gyroscope(new AHRS(SPI.Port.kMXP), true);                                        // Very useful helper class that can invert the gyroscope (which is used to provide the angle of the robot heading to the odometry object)                                                                   // provides angle to odometry object

  // Telemetry 
  SBTab tab = new SBTab("Drive Subsystem");
  private Boolean isRunning;
  private int counter;
  Field2d field = new Field2d();

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
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    simAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    
    populateTab();
    tab.getTab().add(field);
    setPosition(Constants.Drive.START_POSITION);
    setDefaultCommand(new ArcadeDrive(this, ()->Controllers.driverController.getMoveAxis(), ()->Controllers.driverController.getRotateAxis()));
  }

  //Creates DriveSystem ShuffleBoard Tab
   private void populateTab(){
    SBNumberGroup angle = tab.getNumberGroup("angle", 0)
    .setPosition(0,0)
    .setWidth(10)
    .setPeriodic(()->gyro.getRotation2d().getDegrees());

    SBNumberGroup ctsAngle = tab.getNumberGroup("cts angle", 0)
    .setPosition(10,0)
    .setWidth(10)
    .setPeriodic(()->gyro.getContinuousAngle());
    SBNumber leftEncoder = tab.getNumber("Left Encoder VEL", 0)
    .setPosition(20,0)
    .setSize(5,5)
    .setPeriodic(()->getLeftVel());
    SBNumber rightEncoder = tab.getNumber("Right Encoder VEL", 0)
    .setPosition(20,5)
    .setSize(5,5)
    .setPeriodic(()->getRightVel());
    SBBoolean zeroEncoder = tab.getBoolean("Zero Encoders", false)
    .setPosition(20, 10)
    .setSize(5,5)
    .setView(BuiltInWidgets.kToggleButton)
    .setPeriodic((value)->{
      if(value){
      setPosition(Constants.Drive.ZERO_POSITION);
      return false;
    }else{
      return value;
    }});
  }

  
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

  // Encoder-related methods
  public double getRightPos(){
    double meanEncoderVal = (rfMotor.getSelectedSensorPosition() + rbMotor.getSelectedSensorPosition()) / 2; // average of right side native encoder units
    double rotations = meanEncoderVal / Constants.Drive.TALON_UNITS_PER_ROTATION;                       // convert native units to rotations
    double wheelRotations = rotations / Constants.Drive.SHAFT_TO_WHEEL_GEAR_RATIO;                      // convert rotations of motor shaft to rotations of wheel
    double linearDisplacement = wheelRotations * Constants.Drive.WHEEL_CIRCUMFERENCE;                   // convert wheel rotations to linear displacement
    return linearDisplacement;
  }

  public double getLeftPos(){
    double meanEncoderVal = (lfMotor.getSelectedSensorPosition() + lbMotor.getSelectedSensorPosition()) / 2; // average of right side native encoder units
    double rotations = meanEncoderVal / Constants.Drive.TALON_UNITS_PER_ROTATION;                       // convert native units to rotations
    double wheelRotations = rotations / Constants.Drive.SHAFT_TO_WHEEL_GEAR_RATIO;                      // convert rotations of motor shaft to rotations of wheel
    double linearDisplacement = wheelRotations * Constants.Drive.WHEEL_CIRCUMFERENCE;                   // convert wheel rotations to linear displacement
    return linearDisplacement;
  }

  public double getRightVel(){
    double meanEncoderVal = (rfMotor.getSelectedSensorVelocity() + rbMotor.getSelectedSensorVelocity()) / 2; // average of right side native encoder units
    meanEncoderVal *= 10;
    double rotations = meanEncoderVal / Constants.Drive.TALON_UNITS_PER_ROTATION;                       // convert native units to rotations
    double wheelRotations = rotations / Constants.Drive.SHAFT_TO_WHEEL_GEAR_RATIO;                      // convert rotations of motor shaft to rotations of wheel
    double linearDisplacement = wheelRotations * Constants.Drive.WHEEL_CIRCUMFERENCE;                   // convert wheel rotations to linear displacement
    return linearDisplacement;
  }

  public double getLeftVel(){
    double meanEncoderVal = (lfMotor.getSelectedSensorVelocity() + lbMotor.getSelectedSensorVelocity()) / 2; // average of right side native encoder units
    meanEncoderVal *= 10;
    double rotations = meanEncoderVal / Constants.Drive.TALON_UNITS_PER_ROTATION;                       // convert native units to rotations
    double wheelRotations = rotations / Constants.Drive.SHAFT_TO_WHEEL_GEAR_RATIO;                      // convert rotations of motor shaft to rotations of wheel
    double linearDisplacement = wheelRotations * Constants.Drive.WHEEL_CIRCUMFERENCE;                   // convert wheel rotations to linear displacement
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

    var drivetrain = LinearSystemId.identifyDrivetrainSystem(Constants.Drive.Sim.LINEAR_KV, Constants.Drive.Sim.LINEAR_KA, Constants.Drive.Sim.ANGULAR_KV, Constants.Drive.Sim.ANGULAR_KA);
    sim  = new DifferentialDrivetrainSim(drivetrain, DCMotor.getFalcon500(2), Constants.Drive.SHAFT_TO_WHEEL_GEAR_RATIO, Constants.Drive.TRACK_WIDTH, Constants.Drive.WHEEL_RADIUS, null);
  }

  // Gyroscope-related methods
  public double getContinuousGyroAngle(){
    return gyro.getContinuousAngle();
  }

  public void updateOdometry(){
    odometry.update(gyro.getRotation2d(), getLeftPos(), getRightPos());
  }
  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds toWheelSpeeds(ChassisSpeeds chassisSpeeds){
    return kinematics.toWheelSpeeds(chassisSpeeds);
  }
  // Drive-related methods
  public void setRawSpeeds(DifferentialDriveWheelSpeeds speeds){
    speeds.rightMetersPerSecond  = MathUtil.clamp(speeds.rightMetersPerSecond, -12,12);
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

  public DifferentialDriveWheelSpeeds calculateFF(DifferentialDriveWheelSpeeds speeds){
    double rightFFSpeed = rightFeedForward.calculate(speeds.rightMetersPerSecond);
    double leftFFSpeed = leftFeedForward.calculate(speeds.leftMetersPerSecond);

    return new DifferentialDriveWheelSpeeds(leftFFSpeed, rightFFSpeed);
  }

  // PID-related methods
  private void setMotorPID(WPI_TalonFX motor, double kP, double kI, double kD, double allowableError){ // Just makes the setPID() method more readable
    motor.config_kP(0, kP);
    motor.config_kI(0, kI);
    motor.config_kD(0, kD);
    motor.configAllowableClosedloopError(0, allowableError, 0);

  }
  public void setPID(double kP, double kI, double kD, double allowableError){
    setMotorPID(rfMotor, kP, kI, kD, allowableError);
    setMotorPID(lfMotor, kP, kI, kD, allowableError);
  }

  // Acceleration-related methods
  public void setRampRate(double rampRate){  // Ramp rate is how much time it takes to reach max motor speed
    rfMotor.configOpenloopRamp(rampRate);
    lfMotor.configOpenloopRamp(rampRate);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    updateOdometry(); //Must be continuously called to maintain an accurate position of the robot
    tab.periodic();
    field.setRobotPose(odometry.getPoseMeters());
  }

  private int distanceToNativeUnits(double position) { // Necessary Conversions for simulation
    double wheelRotations = position / Constants.Drive.WHEEL_CIRCUMFERENCE;
    double motorRotations = wheelRotations * Constants.Drive.SHAFT_TO_WHEEL_GEAR_RATIO;
    double nativeUnits = motorRotations * Constants.Drive.TALON_UNITS_PER_ROTATION;
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
