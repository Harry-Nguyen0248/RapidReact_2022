// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Indexer.Conveyor;
import frc.robot.subsystems.Indexer.Elevator;

public class RunIndexer extends CommandBase {

  private Conveyor conveyor;
  private Elevator elevator;
  private final boolean up;
  private boolean frontSensorTripped = false;
  private boolean frontSensorTrippedLast = false;

  /** Creates a new runConveyor. */
  public RunIndexer(Conveyor conveyor, Elevator elevator, boolean up) {
    this.conveyor = conveyor;
    this.elevator = elevator;
    this.up = up;
    addRequirements(conveyor, elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (up) {
      conveyor.runConveyor(Constants.Indexer.indexerSpeed);
      elevator.runElevator(Constants.Indexer.indexerSpeed);
    } else {
      conveyor.runConveyor(-Constants.Indexer.indexerSpeed);
      elevator.runElevator(-Constants.Indexer.indexerSpeed);
      frontSensorTrippedLast = conveyor.getFrontSensor();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.getBoolean("up", up);
    if (!up) {
      frontSensorTripped = conveyor.getFrontSensor();
      if(!frontSensorTripped && frontSensorTrippedLast) { RobotContainer.ballCount = Math.max(0, RobotContainer.ballCount - 1); }
      frontSensorTrippedLast = frontSensorTripped;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyor.runConveyor(0);
    elevator.runElevator(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
