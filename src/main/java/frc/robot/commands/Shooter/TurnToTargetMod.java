// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSub.DriveTrain;
import frc.robot.subsystems.VisionSub.Vision;
import frc.robot.util.TuningTable;

public class TurnToTargetMod extends CommandBase {
  private static final TuningTable kP = new TuningTable("AutoAim_kP");
  private static final TuningTable kI = new TuningTable("AutoAim_kI");
  private static final TuningTable kD = new TuningTable("AutoAim_kD");
  private static final TuningTable toleranceDegrees = new TuningTable("AutoAim_toleranceDegrees");
  private static final TuningTable toleranceTime = new TuningTable("toleranceTime");
  private static final TuningTable turnSpeedMod = new TuningTable("AutoAim_turnSpeedMod");

  private final DriveTrain dt;
  private final Vision ll;
  
  private final PIDController controller;
  private final Timer toleranceTimer = new Timer();
  

  /** Creates a new TurnToTargetMod. */
  public TurnToTargetMod(DriveTrain dt, Vision ll) {
    //TODO: values TBD
    kP.setDefault(Constants.AutoAim.autoAimGains.kP);
    kI.setDefault(Constants.AutoAim.autoAimGains.kI);
    kD.setDefault(Constants.AutoAim.autoAimGains.kD);
    toleranceDegrees.setDefault(Constants.AutoAim.toleranceDegrees);
    toleranceTime.setDefault(Constants.AutoAim.toleranceTime);
    turnSpeedMod.setDefault(Constants.AutoAim.turnSpeedMod);

    this.dt = dt;
    this.ll = ll;
    addRequirements(dt);

    controller = new PIDController(kP.get(), kI.get(), kD.get(), Constants.executeDuration);
    controller.setSetpoint(0.0);
    controller.setTolerance(toleranceDegrees.get());
    controller.enableContinuousInput(-180, 180);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.reset();
    toleranceTimer.reset();
    toleranceTimer.start();
    //ll.setLLLed(true);
    // SmartDashboard.putString("Command status", "initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putString("Command status", "executed started");
    if (kP.hasChanged()) {
      controller.setP(kP.get());
    }
    if (kI.hasChanged()) {
      controller.setI(kI.get());
    }
    if (kD.hasChanged()) {
      controller.setI(kD.get());
    }
    if (toleranceDegrees.hasChanged()) {
      controller.setI(toleranceDegrees.get());
    }
    if (toleranceTime.hasChanged()) {
      controller.setI(toleranceTime.get());
    }

    //check if in tolerance
    if(!controller.atSetpoint()){
      toleranceTimer.reset();
    }

    //Calculate angular speed
    double turn = 0.0;
    if(ll.isTargetFound()){
      turn = controller.calculate(ll.getX());
    } 
    // else {
    //   controller.reset();
    // }
    dt.arcadeDrive(0, turn);
    //dt.driveVelocity(-turn * turnSpeedMod.get(), turn * turnSpeedMod.get());
    // SmartDashboard.putString("Command status", "executed ended");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // SmartDashboard.putString("Command status", "command finished");
    dt.stop();
    toleranceTimer.stop();
    //ll.setLLLed(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return toleranceTimer.hasElapsed(toleranceTime.get());
  }
}
