// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain;

import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSub.DriveTrain;
import frc.robot.subsystems.VisionSub.Vision;
import frc.robot.util.OrbitPID;
import frc.robot.util.TuningTable;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TeleopDrivePID extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final DriveTrain dt;
  private final Vision ll;
  private DoubleSupplier turnInput, driveInput;
  private BooleanSupplier autoAim;
  private final OrbitPID turnPID;
  private final TuningTable kP = new TuningTable("kP_Turn");
  private final TuningTable kD = new TuningTable("kD_Turn");

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TeleopDrivePID(DriveTrain dt, Vision ll, DoubleSupplier turnInput, DoubleSupplier driveInput, BooleanSupplier autoAim) {
    this.dt = dt;
    this.ll = ll;
    this.turnInput = turnInput;
    this.driveInput = driveInput;
    this.autoAim = autoAim;
    kP.setDefault(0.5);
    kD.setDefault(0.05);
    this.turnPID = new OrbitPID(kP.get(), 0.0, kD.get(), 0.0, 0.0, 0.0, Constants.DriveTrain.maxTurnOutput, 0.0);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt, ll);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.brakeOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (kP.hasChanged() | kD.hasChanged()) {
      turnPID.configure(kP.get(), 0.0, kD.get(), 0.0, 0.0, 0.0, Constants.DriveTrain.maxTurnOutput, 0.0);
    }
    double turnValue = 0;
    double tx = ll.getX() / Constants.txDegrees;
    SmartDashboard.putNumber("txInPercent", tx);
    SmartDashboard.putBoolean("isAutoAimON", autoAim.getAsBoolean());
    if (autoAim.getAsBoolean()) {
      dt.brakeOn();
      if (Math.abs(tx) > Constants.turnToTargetTolerance) {
        turnValue = turnPID.calculate(0, -(/*Math.abs(turnInput.getAsDouble())* */ tx)); //eliminate the direction of where the joystick is pulled towards to
        dt.arcadeDriveDirect(0, dt.deadband(turnValue));
      } else {
        turnValue = 0;
        dt.arcadeDriveDirect(0, turnValue);
      }
    } else {
      dt.brakeOff();
      double turn = turnInput.getAsDouble();
      dt.arcadeDrive(driveInput.getAsDouble(), turn);
    }
    
    SmartDashboard.putNumber("turnValue", dt.deadband(turnValue));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.brakeOff();
    dt.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
