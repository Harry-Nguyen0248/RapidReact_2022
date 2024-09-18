/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSub.DriveTrain;
import frc.robot.util.OrbitPID;

public class DriveStraight extends CommandBase {

  DriveTrain drive;
  double speed;
  int cyclesTarget, cyclesDone = 0;
  OrbitPID turnGains = new OrbitPID(Constants.driveGains);
  /**
   * Creates a new DriveStraight.
   */
  public DriveStraight(DriveTrain drive, double speed, int cycles) {
    this.drive = drive;
    this.speed = speed;
    this.cyclesTarget = cycles;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    cyclesDone = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putNumber("drivestraight cycle", cyclesDone);
    double turn = turnGains.calculate(0, drive.getHeading());

    drive.arcadeDrive(speed, 0);
    cyclesDone++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
    if(interrupted) {
      // SmartDashboard.putBoolean("drive finished", false);
    } else {
      // SmartDashboard.putBoolean("drive finished", true);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cyclesDone >= cyclesTarget;
  }
}
