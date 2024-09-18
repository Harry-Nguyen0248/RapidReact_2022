// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.util.function.DoubleConsumer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSub.Hood;
import frc.robot.subsystems.ShooterSub.Shooter;
import frc.robot.subsystems.VisionSub.Vision;

public class AutoFlyWheelsPrepared extends CommandBase {
  private final Shooter shooter;
  private final DoubleConsumer rumble;
  private final Hood hood;
  private final Vision ll;
  private boolean rumbleActive = false;
  private boolean lastFlyWheelsReady = false;
  private Timer flyWheelsTimer = new Timer();

  /** Creates a new AimShoot. */
  public AutoFlyWheelsPrepared(Shooter shooter, Vision ll, Hood hood, DoubleConsumer rumble) {
    this.shooter = shooter;
    this.ll = ll;
    this.hood = hood;
    this.rumble = rumble;
    addRequirements(shooter, hood);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hood.setRaised(false);
    ll.setLLLed(true);
    rumbleActive = false;
    lastFlyWheelsReady = shooter.flyWheelsReady();
    flyWheelsTimer.reset();
    flyWheelsTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double ty = ll.getY();
    // if (ll.isLedOn()) {
    // if (ll.isTargetFound()) {
    // if (ty >= 26.5) {
    // shooter.runLoweredHoodVelocity(ty);
    // hood.setRaised(false);
    // } else {
    // shooter.runRaisedHoodVelocity(ty);
    // hood.setRaised(true);
    // }
    // }
    // }

    // constant high shot speed
    shooter.runLoweredHoodVelocity(0);

    // rumble
    boolean flyWheelsReady = shooter.flyWheelsReady();
    if (flyWheelsReady && !lastFlyWheelsReady) {
      rumbleActive = true;
      flyWheelsTimer.reset();
    }
    if (rumbleActive && !flyWheelsTimer.hasElapsed(Constants.rumbleDuration)) {
      rumble.accept(Constants.rumblePercent);
    } else {
      rumble.accept(0.0);
    }
    lastFlyWheelsReady = flyWheelsReady;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.setRaised(false);
    shooter.setVelocity(0, 0);
    rumble.accept(0);
    flyWheelsTimer.stop();
    // ll.setLLLed(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
