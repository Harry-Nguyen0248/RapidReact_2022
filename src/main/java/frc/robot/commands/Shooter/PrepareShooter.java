// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSub.Hood;
import frc.robot.subsystems.ShooterSub.Shooter;
import frc.robot.util.TuningTable;

public class PrepareShooter extends CommandBase {
  private final TuningTable lower_upCloseRpm = new TuningTable ("PrepareShooter_Lower_upCloseRpm");
  private final TuningTable lower_launchPadRpm = new TuningTable("PrepareShooter_Lower_launchPadRpm");
  private final TuningTable upper_upCloseRpm = new TuningTable ("PrepareShooter_Upper_upCloseRpm");
  private final TuningTable upper_launchPadRpm = new TuningTable("PrepareShooter_Upper_launchPadRpm");

  private final Shooter shooter;
  private final ShooterPreset preset;
  private final Hood hood;


  /** Creates a new PrepareShooter. */
  public PrepareShooter(Shooter shooter, Hood hood, ShooterPreset preset) {
    this.shooter = shooter;
    this.hood = hood;
    this.preset = preset;

    // *** TODO: determine the values below
    lower_upCloseRpm.setDefault(Constants.Shooter.lowerFlyWheelUpCloseVel);
    lower_launchPadRpm.setDefault(Constants.Shooter.lowerFlyWheelLaunchPadVel);
    upper_launchPadRpm.setDefault(Constants.Shooter.upperFlyWheelLaunchPadVel);
    upper_upCloseRpm.setDefault(Constants.Shooter.upperFlyWheelUpCloseVel);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean raised = false;
    double flyWheelSpeed = 0.0;
    switch(preset) {
      case UP_CLOSE:
        shooter.setVelocity(lower_upCloseRpm.get(), upper_upCloseRpm.get());
        //hood.setRaised(false);
        break;
      case LAUNCH_PAD:
        shooter.setVelocity(lower_launchPadRpm.get(), upper_launchPadRpm.get());
        // if (shooter.flyWheelsReady()) {
          hood.setRaised(true);
        // }
        break;
      default:
        break;
    }
    // SmartDashboard.putNumber("flyWheelSpeed", flyWheelSpeed);
    // SmartDashboard.putBoolean("isRaised", raised);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.setRaised(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static enum ShooterPreset {
    UP_CLOSE, LAUNCH_PAD
  }
}
