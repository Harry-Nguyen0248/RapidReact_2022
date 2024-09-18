// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Indexer.Conveyor;
import frc.robot.subsystems.Indexer.Elevator;
import frc.robot.subsystems.Indexer.LoaderWheel;
import frc.robot.subsystems.ShooterSub.Shooter;

public class AutoFeedShooter extends CommandBase {
  private final Elevator elevator;
  private final LoaderWheel loader;
  private final Shooter shooter;
  private final Conveyor conveyor;
  private Timer flyWheelsTimer = new Timer();
  private Timer shotTimer = new Timer();
  private boolean firstShotFired = false;
  private boolean sensorLastTripped = false;
  private boolean lastFlyWheelsReady = false;
  private boolean timerReset = false;
  
  /** Creates a new Shoot. */
  public AutoFeedShooter(LoaderWheel loader, Elevator elevator, Conveyor conveyor, Shooter shooter) {
    //led.setShooting(true);
    this.loader = loader;
    this.elevator = elevator;
    this.shooter = shooter;
    this.conveyor = conveyor;

    addRequirements(loader, elevator, conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flyWheelsTimer.reset();
    flyWheelsTimer.start();
    shotTimer.reset();
    shotTimer.start();
    lastFlyWheelsReady = shooter.flyWheelsReady();
    sensorLastTripped = loader.getUpSensor();
    timerReset = false;
    firstShotFired = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timerReset && flyWheelsTimer.hasElapsed(Constants.Shooter.waitToFeed)) { //wait 1s before feeding
      if (!firstShotFired || shotTimer.hasElapsed(Constants.Shooter.shotDelay)) { //delay between 2 shots
        conveyor.runConveyor(Constants.Indexer.indexerSpeed);
        elevator.runElevator(Constants.Indexer.indexerSpeed);
        loader.runLoaderWheel(Constants.Shooter.loaderWheelSpeed);
      } else {
        conveyor.runConveyor(0);
        elevator.runElevator(0);
        loader.runLoaderWheel(0);
      }
    }
    boolean flyWheelsReady = shooter.flyWheelsReady();
    boolean sensorTripped = loader.getUpSensor();

    if(flyWheelsReady && !lastFlyWheelsReady) {
      timerReset = true;
      flyWheelsTimer.reset();
    }

    if (!sensorTripped && sensorLastTripped) {
      firstShotFired = true;
      shotTimer.reset();
      RobotContainer.ballCount = Math.max(0, RobotContainer.ballCount - 1);
    }
    sensorLastTripped = sensorTripped;
    lastFlyWheelsReady = flyWheelsReady;
    SmartDashboard.putBoolean("timerReset", timerReset);
    SmartDashboard.putBoolean("firstShotFired", firstShotFired);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //led.setShooting(false);
    elevator.runElevator(0.0);
    loader.runLoaderWheel(0.0);
    conveyor.runConveyor(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
