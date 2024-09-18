// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot; 


import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.robot.util.Gains;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double rumblePercent = 0.6;
  public static final double rumbleDuration = 0.3;
  public static final double executeDuration = 0.02;

  //TeleopDrivePID
	public static final double turnWeightFactor = 0.3;
	public static final double speedMod = 0.6;
  public static final Gains driveGains = new Gains(0.005,0.0,0.05,0.0,0,0.4);
  public static final double turnToTargetSpeedMod = 5*29;
  public static final double txDegrees = 29.8;
  public static final double turnToTargetTolerance = 0.1;
  public static final double turnToTargetTolDegrees = (txDegrees * turnToTargetTolerance) + 1.0;
  public static final double slowMove = 0.2;
  public static final double slowTurn = 0.2;

  //aiming
  public static final double turretAimSpeedMod = 1/(5*29);
  public static final double turretAimTolerance = 0.5;
public static final double maxVelTurn = 15; //deg/sec
public static final double maxAccTurn = 10; //deg^2/sec
  
  public static final class Ramsete {
  //volage constraints
  public static final double kTrackwidthMeters = 0.642;
  public static final double ksVolts = 0.20517;
  public static final double kvVoltSecondsPerMeter = 2.6019;
  public static final double kaVoltSecondsSquaredPerMeter = 0.56559;
  public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

  //trajectory config
  public static final double kMaxSpeedMetersPerSecond = 6;
  public static final double kMaxAccelerationMetersPerSecondSquared = 2;

  // Ramsete Constants
  public static final double kRamseteZeta = 0.7;
  public static final double kRamseteB = 2;
  public static final double kPDriveVel = 0.01;
  public static final double kIDriveVel = 5.0e-6;
  public static final double maxVoltage = 10;

  public static final LinearSystem<N2, N2, N2> kDriveTrainPlant = LinearSystemId.identifyDrivetrainSystem(
    kvVoltSecondsPerMeter, // Kv of Vs/m
    kaVoltSecondsSquaredPerMeter, // Ka of Vs^2/m
    0.1964, // kv Vs/Ra
    0.027943); // ka Vs^2/Ra
    public static final DCMotor kDriveGearBox = DCMotor.getCIM(2);

  }

  public final static class DriveTrain {
    public static final double driveTrainOpenLoopRampRate = 0.5;
    public static final double wheelRadius = 0;
    public static final double slowLeftTurn = -0.1;
    public static final double slowRightTurn = 0.1;
    public static final double encoderResolution = 42;
    public static final double gearRatio = 9.87;
    public static final double maxTurnOutput = 0.6;
  }

	public final static class OI { // Values TBD
		public final static double deadbandTrigger = 0.1;
		public final static double deadbandJoystick = 0.2;
  }

  public final static class Intake { // Ports & values TBD
    public final static double rollerSpeed = 0.35;
    public final static boolean rollerInvert = true;
    public static final double loaderHoldSpeed = -0.1;
  }
  
  // Shooter constants
  public final static class Shooter {
    // pid constants for aiming
 

    // turret config constants
    public static final float turretSoftLimitForward = 5; // value TBD
    public static final float turretSoftLimitReverse = 0; // value TBD
    public static final double upperShooterVoltage = 0;
    public static final double lowerShooterVoltage = 0;

    //AimShoot
    public static final Gains flyWheelGains = new Gains(6e-5,0.0, 0.0, 0.0, 0, 0.0);
    public static final double flyWheelIdleSpeed = 1000;
    public static final double lowerFlyWheelUpCloseVel = 1700; //1700 
    public static final double lowerFlyWheelLaunchPadVel = 75;
    public static final double upperFlyWheelUpCloseVel = 0; //0 (zero)
    public static final double upperFlyWheelLaunchPadVel = 0;

    public static final double loaderWheelSpeed = 0.5;

    public static final boolean feederInverted = true;
    public static final boolean upperFlyWheelInverted = false;
    public static final boolean lowerFlyWheelInverted = false;
    public static final boolean loaderWheelInverted = true;

    //TODO:Determine the values below
    public static final double rpmRatio = 0.8;
    public static final double ffVoltsRatio = 0;

    public static final double kS_Lower = 0.32929;
    public static final double kV_Lower = 0.12651;
    public static final double kA_Lower = 0.012115;
    public static final double KS_Upper = 0.26608;
    public static final double KV_Upper = 0.13075;
    public static final double kA_Upper = 0.012609;

    public static final int maxRPM = 5000;
    public static final double maxAccelRpm = 1; // TEST VALUE because i suspect 0 may break things
    public static final double velocityMod = 0;

    //hood
    public static final double hoodMovingtime = 0;

    public static final double kP_Lower = 0.00015;
    public static final double kI_Lower = 0;
    public static final double kD_Lower = 0.00012;
    public static final double kFF_Lower = 0.0001839;
    public static final double kP_Upper = 0.000003;
    public static final double kI_Upper = 0;
    public static final double kD_Upper = 0.0005;
    public static final double kFF_Upper = 0.0001865;
    public static final double flyWheelRPMRatio = 0.8;
    public static final double flyWheelClosedLoopRampRate = 0.5;

    public static final double shotDelay = 0.5;
    public static final double upperToleranceRpm = 100;
    public static final double lowerToleranceRpm = 100;
    public static final double waitToFeed = 1.0;
  }

  public final static class Indexer{
    public static final double indexerSpeed = 1.0;

    public static final Color kBlueTarget = new Color(0.15919, 0.39938, 0.441914);
    public static final Color kRedTarget = new Color(0.56626, 0.31396, 0.093261);
    //
    public static Color kTarget = kBlueTarget;
    
    public static final int frontSensor = 0;
    public static final int midSensor = 1;
    public static final int upSensor = 2;
    public static final boolean elevatorInvert = true;
    public static final boolean conveyorInvert = false;
    public static final double indexTimer = 3000;
    public static final double conveyorTimer = 3000;
    public static final int ejectTimer = 3000;
    public static final long elevatorTimer = 1500;
  }

  public final static class Vision {

    public static final Gains targetAlignmentGains = null;
    public static final double cameraMountAngle = 0;
    public static final double targetStructureHeight = 103.5;
    public static final double cameraMountHeight = 38; 

  }

  public final static class Turret {
    public static final double softLimitForward = 0; // TODO: VALUES TBD
    public static final double softLimitBackward = 0;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFF = 0;
    public static final double kIz = 0;
  }

  public final static class AutoAim {
    //TODO: determine the value
    public static final Gains autoAimGains = new Gains(0.05, 0.0 ,0.0, 0.0, 0, 0.0); 
    public static final double toleranceTime = 0.5;
    public static final double toleranceDegrees = 0.3;
    public static final double turnSpeedMod = 0.5;
  }

  public final static class Climber {

    public static final double setLiftTrueTimer = 1.0;

  }

  public final static class RobotMap {
    //Intake
    public static final int sparkRollers = 21;
    public static final int pistonIntake = 0;

    //DriveTrain
    public static final int sparkMasterLeft = 10; 
    public static final int sparkMasterRight = 12;
    public static final int SparkFollowerLeft = 11;
    public static final int SparkFollowerRight = 13;

    // Indexer
    public static final int sparkElevator = 41;
    public static final int sparkConveyor = 31;
    public static final int[] digitalInProxSensors = new int[] {0, 1, 2}; // match these to the values in lowerIndexer
    
    //Intake
    public final static int rollerMotor = 4; // subject to change

    //OI
    public final static int driverController = 0;
    public static final int operatorController = 1;

    //Shooter
    public static final int sparkUpperWheel = 62;
    public static final int sparkLowerWheel = 61;
    public static final int sparkTurret = 51;
    public static final int sparkLoader = 42;

    //LEDs
    public static final int LEDs = 0;

    //Hood
    public static final int pistonHood = 1; 

    //Climber 
    public static final int pistonClimberLatch = 2; // correct value
    public static final int pistonClimberClimbingF = 3; // F and R may be swapped
    public static final int pistonClimberClimbingR = 4;
    public static final int pistonClimberClimbing = 3;
  }
}