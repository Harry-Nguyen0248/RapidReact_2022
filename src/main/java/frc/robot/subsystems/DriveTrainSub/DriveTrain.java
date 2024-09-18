// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrainSub;


import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class DriveTrain extends SubsystemBase {

  //motors
  private final CANSparkMax leftM;
  private final CANSparkMax rightM;
  private final CANSparkMax leftS;
  private final CANSparkMax rightS;


  //gyroscope
  public final AHRS gyro;

  //motor controller
  private final MotorController leftMotors;
  private final MotorController rightMotors;

  //Encoders
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  //drive
  private final DifferentialDrive drive;

  //robot pose
  private final DifferentialDriveOdometry odometry;

  // //feedforward
  // private final SimpleMotorFeedforward leftFF;
  // private final SimpleMotorFeedforward rightFF;

  // //pid controller
  // private final SparkMaxPIDController leftPID;
  // private final SparkMaxPIDController rightPID;


  private double conversion;



  /** Creates a new ExampleSubsystem. */
  public DriveTrain(int leftID, int rightID, int leftSID, int rightSID, MotorType type) {
    gyro = new AHRS(Port.kMXP);
    
    leftM = new CANSparkMax(leftID, type);
    rightM = new CANSparkMax(rightID, type);
    leftS = new CANSparkMax(leftSID, type);
    rightS = new CANSparkMax(rightSID, type);

    leftMotors = new MotorControllerGroup(leftM, leftS);
    rightMotors = new MotorControllerGroup(rightM, rightS);

    conversion = (Math.PI * 6) / Constants.DriveTrain.gearRatio;
    leftEncoder = leftM.getEncoder();
    rightEncoder = rightM.getEncoder();
    leftEncoder.getPositionConversionFactor();
    rightEncoder.getPositionConversionFactor();
    leftEncoder.getVelocityConversionFactor();
    rightEncoder.getVelocityConversionFactor();
    leftEncoder.setPositionConversionFactor(conversion);
    rightEncoder.setPositionConversionFactor(conversion);
    leftEncoder.setVelocityConversionFactor(conversion/60);
    rightEncoder.setVelocityConversionFactor(conversion/60);
    


    drive = new DifferentialDrive(leftMotors, rightMotors);
    drive.setSafetyEnabled(false);
    drive.setExpiration(Robot.kDefaultPeriod);

    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    // leftFF = new SimpleMotorFeedforward(Constants.Ramsete.ksVolts, Constants.Ramsete.kvVoltSecondsPerMeter);
    // rightFF = new SimpleMotorFeedforward(Constants.Ramsete.ksVolts, Constants.Ramsete.kvVoltSecondsPerMeter);

    // leftPID = leftM.getPIDController();
    // rightPID = rightM.getPIDController();
    // configurePID(Constants.driveGains.kP, Constants.driveGains.kI, Constants.driveGains.kD);
  }

  //access pose
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  //reset pose
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  //reset Encoders
  public void resetEncoders() {
    leftM.getEncoder().setPosition(0.0);
    rightM.getEncoder().setPosition(0.0);
  }

  //reset gyro
  public void zeroHeading() {
    gyro.reset();
  }

  //set voltage to motors
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    drive.feed();
  }

  //access encoders' values
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    DifferentialDriveWheelSpeeds speeds = new DifferentialDriveWheelSpeeds(
      Units.inchesToMeters(leftEncoder.getVelocity()*conversion/(60*36)),
      Units.inchesToMeters(rightEncoder.getVelocity())*conversion/(60*36));
    return speeds;
  }
  //get robot's z-axis value
  public double getHeading() {
    return gyro.getYaw();
  }

  public void arcadeDrive(double speed, double turn) {
    double left;
    double right;
    if (turn > 0.0){
      left = (speed) + (fastExp(Constants.turnWeightFactor * turn) * turn);
      right = (speed) + (fastExp(Constants.turnWeightFactor   * turn) * -turn);
    }
    else if (turn < 0.0) {
      left = (speed) + (fastExp(Constants.turnWeightFactor   * -turn) * turn);
      right = (speed) + (fastExp(Constants.turnWeightFactor   * -turn) * -turn);
    }else {
      left = speed;
      right = speed;
    }

    tankDrive(left, right);
  }

  public void arcadeDriveDirect(double speed, double turn)
  {
    double left;
    double right;

    left = speed + turn;
    right = speed + -turn;

    tankDrive(left, right);
  }

  public void tankDrive(double left, double right) {
    leftM.set(left /** Constants.speedMod*/);
    rightM.set(right /** Constants.speedMod*/);
  }

  // /** run closed loop at the specificied velocity */
  // public void setVelocity(double leftRPM, double rightRPM, double leftFFVolts, double rightFFVolts) {
  //   leftPID.setReference(leftRPM, ControlType.kVelocity, 
  //       0, leftFFVolts, ArbFFUnits.kVoltage);
  //   rightPID.setReference(rightRPM, ControlType.kVelocity, 
  //       0, rightFFVolts, ArbFFUnits.kVoltage);
  // }

  // /** drive at the specified velocity. */
  // public void driveVelocity(double leftRPM, double rightRPM) {
  //   double leftRad = leftRPM / Constants.DriveTrain.wheelRadius;
  //   double rightRad = rightRPM / Constants.DriveTrain.wheelRadius;

  //   double leftFFVolts = leftFF.calculate(leftRad);
  //   double rightFFVolts = rightFF.calculate(rightRad);

  //   setVelocity(leftRPM, rightRPM, leftFFVolts, rightFFVolts);
  // }

  /** Stop by going to open loop. */
  public void stop(){
    leftM.setVoltage(0);
    rightM.setVoltage(0);
  }

  private double fastExp(double x) {

    x = 1d + x/256d;

    for (int i = 0; i< 8; i++){
      x *= x;
    }
    return x;

  }

  @Override
  public void periodic() {

    if (rightM.getInverted() != DriveTrainConfig.isRightInverted) {
      rightM.setInverted(DriveTrainConfig.isRightInverted);
    }
    if (rightS.getInverted() != DriveTrainConfig.isRightInverted) {
      rightS.setInverted(DriveTrainConfig.isRightInverted);
    }
    //update pose
    odometry.update(
      Rotation2d.fromDegrees(getHeading()), 
      Units.inchesToMeters(leftEncoder.getPosition()*conversion),
      Units.inchesToMeters(rightEncoder.getPosition())*conversion);

    SmartDashboard.putNumber("Right Meter", Units.inchesToMeters(rightEncoder.getPosition())*conversion);
    SmartDashboard.putNumber("Left Meter", Units.inchesToMeters(leftEncoder.getPosition())*conversion);
    SmartDashboard.putNumber("Heading angle", gyro.getAngle());
    SmartDashboard.putNumber("Yaw value", gyro.getYaw());
    SmartDashboard.putNumber("Pitch value", gyro.getPitch());
    SmartDashboard.putNumber("Roll value", gyro.getRoll());
    SmartDashboard.putNumber("Left Vel ", leftEncoder.getVelocity()*conversion/(60*36));
    SmartDashboard.putNumber("Right Vel", rightEncoder.getVelocity()*conversion/(36*60));
    SmartDashboard.putNumber("X-coor", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Y-coor", odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Rotation2dAngle", Rotation2d.fromDegrees(getHeading()).getDegrees());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  //teleop config
  public void configTelelop() {
    DriveTrainConfig.configDriveTeleop(leftM, rightM, leftS, rightS);
  }

  //auto config 
  public void configAuto() {
    DriveTrainConfig.configDriveAuto(leftM, rightM, leftS, rightS, gyro);
  }

  //disable tankdrive
  public void disabled() {
    drive.tankDrive(0, 0);
    DriveTrainConfig.driveDisabled(leftM, rightM, leftS, rightS);
  }

  public void brakeOn() {
    rightM.setIdleMode(IdleMode.kBrake);
    leftM.setIdleMode(IdleMode.kBrake);
    rightS.setIdleMode(IdleMode.kBrake);
    leftS.setIdleMode(IdleMode.kBrake);
  }

  public void brakeOff() {
    rightM.setIdleMode(IdleMode.kCoast);
    leftM.setIdleMode(IdleMode.kCoast);
    rightS.setIdleMode(IdleMode.kCoast);
    leftS.setIdleMode(IdleMode.kCoast);
  }

  public double deadband(double turnValue) {
    return Math.abs(turnValue) < Constants.DriveTrain.maxTurnOutput? turnValue : Math.copySign(Constants.DriveTrain.maxTurnOutput, turnValue);
  }


  // //configure pid for motor controllers
  // public void configurePID(double kP, double kI, double kD) {
  //     leftPID.setP(kP);
  //     leftPID.setI(kI);
  //     leftPID.setD(kD);
  //     leftPID.setFF(0);

  //     rightPID.setP(kP);
  //     rightPID.setI(kI);
  //     rightPID.setD(kD);
  //     rightPID.setFF(0);
  // }
}
