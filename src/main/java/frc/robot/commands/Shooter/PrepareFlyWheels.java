// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;



import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSub.Hood;
import frc.robot.subsystems.ShooterSub.Shooter;
import frc.robot.subsystems.VisionSub.Vision;

public class PrepareFlyWheels extends CommandBase {
  Shooter shooter;
  Vision ll;
  Hood hood;
  double targetLowerVelocity, targetUpperVelocity;

  /** Creates a new AimShoot. */
  public PrepareFlyWheels(Shooter shooter, Vision ll, Hood hood) {
    this.shooter = shooter;
    this.hood = hood;
    this.ll = ll;
    addRequirements(shooter, ll, hood);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hood.setRaised(false);
    ll.setLLLed(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetLowerVelocity = SmartDashboard.getNumber("targetLowerVelocity", 0);
    targetUpperVelocity = SmartDashboard.getNumber("targetUpperVelocity", 0);
    shooter.setVelocity(targetLowerVelocity, targetUpperVelocity);
    if (shooter.flyWheelsReady()) {
      hood.setRaised(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.setRaised(false);
    ll.setLLLed(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
