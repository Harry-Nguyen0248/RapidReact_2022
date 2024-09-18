// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.util.function.DoubleConsumer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Indexer.Elevator;
import frc.robot.subsystems.Indexer.LoaderWheel;

public class FeedShooter extends CommandBase {

  private final Elevator elevator;
  private final LoaderWheel loader;

  private final DoubleConsumer rumble;
  private boolean sensorLastTripped = false;
  private boolean rumbleActive = false;
  private boolean firstShotFired = false;
  private final Timer shotTimer = new Timer();
  
  /** Creates a new Shoot. */
  public FeedShooter(LoaderWheel loader, Elevator elevator, DoubleConsumer rumble) {
    //led.setShooting(true);
    this.loader = loader;
    this.elevator = elevator;
    this.rumble = rumble;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(loader, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //leds
    sensorLastTripped = loader.getUpSensor();
    rumbleActive = false;
    firstShotFired = false;
    shotTimer.reset();
    shotTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shotTimer.hasElapsed(Constants.Shooter.shotDelay) || !firstShotFired) {
      elevator.runElevator(Constants.Indexer.indexerSpeed);
      loader.runLoaderWheel(Constants.Indexer.indexerSpeed);
    }
    else {
      elevator.runElevator(0);
      loader.runLoaderWheel(0);
    }

    boolean sensorTripped = loader.getUpSensor();
    if (sensorLastTripped && !sensorTripped) {
      // rumble the console if the ball leaves successfully
      rumbleActive = true;
      firstShotFired = true;
      shotTimer.reset();
      RobotContainer.ballCount = Math.max(0, RobotContainer.ballCount - 1);
      elevator.setElevatorFull(false);
    }
    if(rumbleActive && !shotTimer.hasElapsed(Constants.rumbleDuration)) {
      rumble.accept(Constants.rumblePercent);
    } else {
      rumble.accept(0.0);
    }

    SmartDashboard.putBoolean("1st shot", firstShotFired);
    SmartDashboard.putBoolean("time", shotTimer.hasElapsed(Constants.Shooter.shotDelay));

    sensorLastTripped = sensorTripped;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //led.setShooting(false);
    rumble.accept(0.0);
    shotTimer.stop();
    elevator.runElevator(0.0);
    loader.runLoaderWheel(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
