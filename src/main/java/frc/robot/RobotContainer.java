// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.Auto.RamseteStorage;
import frc.robot.commands.Climber.ExtendClimber;
import frc.robot.commands.DriveTrain.SlowDriveTurn;
import frc.robot.commands.DriveTrain.TeleopDrivePID;
import frc.robot.commands.DriveTrain.SlowDriveTurn.Directions;
import frc.robot.commands.Indexer.RunIndexer;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.LEDs.LedCommand;
import frc.robot.commands.Shooter.KeepShooterWarm;
import frc.robot.commands.Shooter.PrepareShooter;
import frc.robot.commands.Shooter.TurnToTargetTest;
import frc.robot.commands.Shooter.AutoFlyWheelsPrepared;
import frc.robot.commands.Shooter.FeedShooter;
import frc.robot.commands.Shooter.PrepareShooter.ShooterPreset;
import frc.robot.subsystems.ClimberSub.Climber;
import frc.robot.subsystems.DriveTrainSub.DriveTrain;
import frc.robot.subsystems.Indexer.*;
import frc.robot.subsystems.IntakeSub.Intake;
import frc.robot.subsystems.LEDsSub.LEDs;
import frc.robot.subsystems.ShooterSub.Hood;
import frc.robot.subsystems.ShooterSub.Shooter;
import frc.robot.subsystems.VisionSub.Vision;
import frc.robot.util.LedValues.Colour1And2Pattern;
import frc.robot.util.LedValues.Colour2Pattern;
import frc.robot.util.LedValues.FixedPalettePattern;
import frc.robot.util.LedValues.SolidColour;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain dt = new DriveTrain(Constants.RobotMap.sparkMasterLeft,
                                              Constants.RobotMap.sparkMasterRight,
                                              Constants.RobotMap.SparkFollowerLeft,
                                              Constants.RobotMap.SparkFollowerRight,
                                              MotorType.kBrushless);
  private final OI oi = new OI();
  
  private final Intake intake = new Intake(Constants.RobotMap.sparkRollers, 
                                          MotorType.kBrushless, 
                                          PneumaticsModuleType.CTREPCM,
                                          Constants.RobotMap.pistonIntake); 

  private final Shooter shooter = new Shooter(Constants.RobotMap.sparkUpperWheel,
                                            Constants.RobotMap.sparkLowerWheel,
                                            MotorType.kBrushless);
  
  // private final Turret turret = new Turret(Constants.RobotMap.sparkTurret);
  
  private final Conveyor conveyor = new Conveyor(Constants.RobotMap.sparkConveyor,
                                                Constants.Indexer.frontSensor,
                                                 MotorType.kBrushless);

  private final Elevator elevator = new Elevator(Constants.RobotMap.sparkElevator, 
                                                Constants.Indexer.midSensor,
                                                MotorType.kBrushless);

  private final LoaderWheel loaderWheel = new LoaderWheel(Constants.RobotMap.sparkLoader,
                                                          Constants.Indexer.upSensor,
                                                           MotorType.kBrushless);

  private final Hood hood = new Hood(PneumaticsModuleType.CTREPCM, Constants.RobotMap.pistonHood);

  private final Vision ll = new Vision();

  private final Climber climber = new Climber(Constants.RobotMap.pistonClimberLatch, Constants.RobotMap.pistonClimberClimbing); 

  private final LEDs leds = new LEDs();

  // private final PneumaticHub pneumaticHub = new PneumaticHub();
  private final PowerDistribution pdpHub = new PowerDistribution(2, PowerDistribution.ModuleType.kRev);
  private final Solenoid solenoidPH = new Solenoid(PneumaticsModuleType.CTREPCM, 7);

  private final TeleopDrivePID pidDrive = new TeleopDrivePID(this.dt, this.ll, () -> oi.getDriveTurn(), () -> oi.getDriveThrottle(), oi.getTurnToTargetButton());
  //private final TurnToTarget turnToTarget = new TurnToTarget(ll, dt);

  public LedCommand ledCommand = new LedCommand(leds);

  public static int ballCount = 1;

  // Autos
  private RamseteStorage ramseteStorage = new RamseteStorage(dt, intake, elevator, conveyor, loaderWheel, shooter, hood, ll);
  private Command fiveBallAuto = ramseteStorage.FiveCargoAuto();
  private Command fourBallAuto = ramseteStorage.FiveCargoAuto();
  private Command threeBallAuto = ramseteStorage.FiveCargoAuto();
  private Command twoBallAuto = ramseteStorage.TwoCargoAuto();
  private Command oneBallAuto = ramseteStorage.OneCargoAuto();
  // private Command testAuto = ramseteStorage.TestAuto(); 

  private TurnToTargetTest turnToTargetTest = new TurnToTargetTest(dt, ll);

  private SendableChooser<Command> autoChooser = new SendableChooser<>();
  //private final DriveStraight driveStraight = new DriveStraight(this.dt, 0.5, 50);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    setLEDs();
    // Configure the button bindings
    setDefaultCommands();
    configureButtonBindings();

    autoChooser.setDefaultOption("five balls", fiveBallAuto);
    autoChooser.addOption("four balls", fourBallAuto);
    autoChooser.addOption("three balls", threeBallAuto);
    autoChooser.addOption("two balls", twoBallAuto);
    autoChooser.addOption("one balls", oneBallAuto);

    //ll.setLLLed(true);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  // *** DRIVER CONTROLS ***

  //oi.getTurnToTargetButton().whileActiveOnce(turnToTarget); //TODO: Try if TurnToTargetMod works better
  oi.getSlowForwards().whileActiveOnce(new SlowDriveTurn(dt, Directions.FORWARDS));
  oi.getSlowBackwards().whileActiveOnce(new SlowDriveTurn(dt, Directions.BACKWARDS));
  oi.getSlowLeftTurn().whileActiveOnce(new SlowDriveTurn(dt, Directions.LEFT));
  oi.getSlowRightTurn().whileActiveOnce(new SlowDriveTurn(dt, Directions.RIGHT));
  
  // oi.getClimberArmsExtensionButton().whenActive(new ExtendClimber(climber));
  // oi.getClimberLiftRobotButton().whenActive(new RaiseRobot(climber));
  oi.getClimbMode().and(oi.getClimberArmsExtensionButton()).whenActive(new ExtendClimber(climber));
  Trigger readyToLift = new Trigger (climber::getClimbingPistonState);
  Trigger toggleLift = readyToLift.negate();
  oi.getClimbMode().and(oi.getClimberLiftRobotButton()).and(readyToLift).whenActive(new InstantCommand(climber::liftRobot));
  oi.getClimbMode().and(oi.getClimbingArmExtendButton()).and(toggleLift).whenActive(new InstantCommand(climber::setLiftPistonOn));

  //OI for climb
  // Trigger climbMode = oi.getClimbMode();
  // oi.getClimberArmsExtensionButton().and(climbMode).whenActive(climber::extendClimberArms, climber);
  // *** OPERATOR CONTROLS ***

  // OI for intake
  oi.getIntakeForwardsTrigger().whileActiveContinuous(
      new RunIntake(intake, conveyor, elevator, loaderWheel, oi::setOperatorRumble));
  
  //OI for manual indexer
  oi.getManualIndexerBackward().whileActiveContinuous(
          new RunIndexer(conveyor, elevator,false));
  oi.getManualIndexerForward().whileActiveContinuous(
          new RunIndexer(conveyor, elevator,true));


  //OI for shooter
  oi.getFlyWheelsPreparedTrigger().whileActiveOnce(new AutoFlyWheelsPrepared(shooter, ll, hood, oi::setOperatorRumble));

  // oi.getTurretLeftTrigger().whileActiveOnce(new ManualTurret(turret, true, false));
  // oi.getTurretRightTrigger().whileActiveOnce(new ManualTurret(turret, false, true));

  oi.getBallCountResetTrigger().whileActiveOnce(new InstantCommand(() -> ballCount = 0));

  Command upCloseShootCommand = new PrepareShooter(shooter, hood, ShooterPreset.UP_CLOSE);
  Command launchPadShootCommand = new PrepareShooter(shooter, hood, ShooterPreset.LAUNCH_PAD);

  oi.getUpCloseShootButton().whenActive(upCloseShootCommand);
  oi.getLaunchPadShootButton().whenActive(launchPadShootCommand);
  oi.getStopFlyWheelButotn().cancelWhenActive(upCloseShootCommand)
                            .cancelWhenActive(launchPadShootCommand);

  oi.getFeedShooterButton().whileActiveOnce(
    new FeedShooter(loaderWheel, elevator, oi::setOperatorRumble));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
      return oneBallAuto;
      // return new SequentialCommandGroup(turnToTargetTest);
      // return turnToTargetTest;
  }

  public void setDefaultCommands() {
    dt.setDefaultCommand(pidDrive);
    shooter.setDefaultCommand(new KeepShooterWarm(shooter));
    leds.setDefaultCommand(ledCommand);
  }
  
  public DriveTrain getDriveTrain() {
    return dt;
  }

  public PneumaticHub getPneumaticHub() {
    return null; //pneumaticHub;
  }

  public PowerDistribution getPdpHub() {
    return pdpHub;
  }  

  public void setSolenoidPH(boolean set) {
    // SmartDashboard.putBoolean("solenoid7", set);
    solenoidPH.set(set);
  }

  public void setLLLeds(boolean a) { ll.setLLLed(a); }
  
  private Trigger kachow = oi.getKachow();

  //feedback for cargo count
  private Trigger elevatorHalfFull = new Trigger(() -> ballCount == 1);
  private Trigger elevatorFull = new Trigger(() -> ballCount == 2);
  private Trigger cargoLimitViolation = new Trigger(() -> ballCount > 2);

  //feedback for shooting
  private Trigger txWithinTol = new Trigger(() -> Math.abs(ll.getX()) < Constants.turnToTargetTolDegrees);
  private Trigger targetFound = new Trigger(ll::isTargetFound);
  private Trigger onTarget = txWithinTol.and(targetFound);
  private Trigger offTarget = onTarget.negate();
  private Trigger currentlyShooting = oi.getFeedShooterButton();
  private Trigger flyWheelsReady = new Trigger(shooter::flyWheelsReady);
  private Trigger readyToShoot = flyWheelsReady.and(onTarget);
  private Trigger onlyFlywheelsReady = flyWheelsReady.and(offTarget);


  //feedback for climbing
  private Trigger climberOn = oi.getClimbMode();
  private Trigger hasLifted = new Trigger(climber::getClimbingPistonState);
  private Trigger readyToLift = climberOn.and(hasLifted.negate());

  private void setLEDs() {
    // register all led triggers
    ledCommand.addTrigger(hasLifted,            Colour2Pattern.HEARTBEAT_FAST.get(),                       3);
    ledCommand.addTrigger(readyToLift,          Colour2Pattern.LIGHT_CHASE.get(),                    4);
    ledCommand.addTrigger(kachow,               FixedPalettePattern.COLOR_WAVES__LAVA_PALETTE.get(), 5);
    ledCommand.addTrigger(cargoLimitViolation,  FixedPalettePattern.LIGHT_CHASE__RED.get(),          6);
    ledCommand.addTrigger(currentlyShooting,    SolidColour.WHITE.get(),                             7);
    ledCommand.addTrigger(readyToShoot,         SolidColour.HOT_PINK.get(),                          8);
    // ledCommand.addTrigger(onlyFlywheelsReady,   SolidColour.VIOLET.get(),                            9);
    ledCommand.addTrigger(onTarget,             SolidColour.BLUE.get(),                              10);
    ledCommand.addTrigger(elevatorFull,         FixedPalettePattern.STROBE__GOLD.get(),              11);
    ledCommand.addTrigger(elevatorHalfFull,     SolidColour.GREEN.get(),                            12);
    ledCommand.addTrigger(intake::getPistonPos, SolidColour.AQUA.get(),                              13);
    ledCommand.addTrigger(DriverStation::isAutonomous, FixedPalettePattern.HEARTBEAT__WHITE.get(),      99);
    ledCommand.addTrigger(() -> true, SolidColour.BLACK.get(), 100); // lowest priority 
  }
}
