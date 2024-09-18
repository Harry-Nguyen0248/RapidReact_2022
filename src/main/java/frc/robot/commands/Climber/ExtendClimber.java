// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSub.Climber;
import frc.robot.util.TuningTable;

public class ExtendClimber extends CommandBase {
  private final Climber climber;
  // private final BooleanSupplier latch;
  // private final BooleanSupplier lift;
  // private boolean hasLatched;
  boolean readyToLift, hasLatched;
  private final TuningTable setLiftTrueTime = new TuningTable("setLiftTrueTimer");
  private Timer setLiftTrueTimer = new Timer();
  // private Trigger liftTrigger;
  /** Creates a new Climb. */
  public ExtendClimber(Climber climber) {
    this.climber = climber;
    // this.latch = latch;
    // this.lift = lift;
    setLiftTrueTime.setDefault(Constants.Climber.setLiftTrueTimer);
    // liftTrigger = new Trigger(lift);
    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasLatched = false;
    readyToLift = false;
    setLiftTrueTimer.reset();
    setLiftTrueTimer.start();
    SmartDashboard.putString("TimeElasped", "Initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("TimeElasped", "Command Started");
    if (!hasLatched) {
      climber.extendClimberArms();
      setLiftTrueTimer.reset();
      hasLatched = true;
    }  
    if (hasLatched && setLiftTrueTimer.hasElapsed(setLiftTrueTime.get())) { 
      climber.setLiftPistonOn();
      readyToLift = true;
      SmartDashboard.putString("TimeElasped", "Command Running");
    }
    // if (hasLatched && readyToLift) {
    //   liftTrigger.whenActive(climber::liftRobot);
    // }
    // SmartDashboard.putBoolean("pistonLatched", hasLatched);
    SmartDashboard.putBoolean("readyToLift", readyToLift);
    SmartDashboard.putString("TimeElasped", "Command Ended");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("TimeElasped", "Ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return readyToLift;
  }
}
