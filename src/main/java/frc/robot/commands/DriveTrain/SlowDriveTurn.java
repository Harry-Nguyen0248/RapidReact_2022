// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSub.DriveTrain;

public class SlowDriveTurn extends CommandBase {
  public enum Directions {
    LEFT, RIGHT, FORWARDS, BACKWARDS
  }
  
  private final DriveTrain dt;
  private final Directions dir;

  /** Creates a new SlowDriveTurn. */
  public SlowDriveTurn(DriveTrain dt, Directions dir) {
    this.dt = dt;
    this.dir = dir;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.brakeOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(dir) {
      case LEFT:
        dt.arcadeDriveDirect(0, -Constants.slowTurn);
        break;
      case RIGHT:
        dt.arcadeDriveDirect(0, Constants.slowTurn);
        break;
      case FORWARDS:
        dt.arcadeDriveDirect(Constants.slowMove, 0);
        break;
      case BACKWARDS:
        dt.arcadeDriveDirect(-Constants.slowMove, 0);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.arcadeDriveDirect(0, 0);
    dt.brakeOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
