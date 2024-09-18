// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSub.DriveTrain;
import frc.robot.subsystems.VisionSub.Vision;
import frc.robot.util.OrbitPID;

public class TurnToTargetTest extends CommandBase {

  private final DriveTrain dt;
  private final Vision ll;
  private boolean isOnTargetTest;
  private final OrbitPID turnPID;
  private Timer toleranceTimer = new Timer();
  /** Creates a new TurnToTargetTest. */
  public TurnToTargetTest(DriveTrain dt, Vision ll) {
    this.dt = dt;
    this.ll = ll;
    turnPID = new OrbitPID(0.5, 0.0, 0.05, 0.0, 0.0, 0.0, 0.6, 0.0);
    addRequirements(dt, ll);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.brakeOn();
    isOnTargetTest = false;
    toleranceTimer.reset();
    toleranceTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("turnToTargetCommand", "execute start");
    if (!isOnTargetTest) {
      toleranceTimer.reset();
    }
    double turnValue = 0;
    double tx = ll.getX() / Constants.txDegrees;
    if (ll.isLedOn() && ll.isTargetFound()) {
      dt.brakeOn();
      if (Math.abs(tx) > Constants.turnToTargetTolerance) {
        turnValue = turnPID.calculate(0, -(/*Math.abs(turnInput.getAsDouble())* */ tx)); //eliminate the direction of where the joystick is pulled towards to
        dt.arcadeDriveDirect(0, dt.deadband(turnValue));
      } else {
        turnValue = 0;
        dt.arcadeDriveDirect(0, turnValue);
        isOnTargetTest = true;
      }
    }
    SmartDashboard.putNumber("toleranceTimer", toleranceTimer.get());
    SmartDashboard.putBoolean("isOnTargetTest", isOnTargetTest);
    SmartDashboard.putString("turnToTargetCommand", "execute end");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.brakeOff();
    dt.stop();
    isOnTargetTest = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return toleranceTimer.hasElapsed(Constants.AutoAim.toleranceTime);
  }
}
