// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import java.util.function.DoubleConsumer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Indexer.Conveyor;
import frc.robot.subsystems.Indexer.Elevator;
import frc.robot.subsystems.Indexer.LoaderWheel;
import frc.robot.subsystems.IntakeSub.Intake;

public class RunIntake extends CommandBase {

  Intake intake;
  Conveyor conveyor;
  Elevator elevator;
  LoaderWheel loader;
  DoubleConsumer rumble;

  private boolean rumbleLastOneTripped = false;
  private boolean rumbleLastTwoTripped = false;
  private boolean rumbleOneActive = false;
  private boolean rumbleTwoActive = false;
  private boolean frontSensorTrippedLast = false;
  private boolean frontSensorTrippedActive = false;
  private int lastCount = 0;
  private final Timer rumbleOneTimer = new Timer();
  private final Timer rumbleTwoTimer = new Timer();
  private final Timer frontSensorTolTimer = new Timer();

  /** Creates a new RunIntake. */
  public RunIntake(Intake intake, Conveyor conveyor, Elevator elevator, LoaderWheel loader, DoubleConsumer rumble) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.conveyor = conveyor;
    this.elevator = elevator;
    this.loader = loader;
    this.rumble = rumble;

    addRequirements(intake, conveyor, elevator, loader);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    intake.setIntake(false);
    
    rumbleLastOneTripped = loader.getUpSensor();
    rumbleLastTwoTripped = loader.getUpSensor() && elevator.getMidSensor();
    rumbleOneActive = false;
    rumbleTwoActive = false;

    frontSensorTrippedLast = conveyor.getFrontSensor();
    lastCount = RobotContainer.ballCount;

    rumbleOneTimer.reset();
    rumbleTwoTimer.reset();
    rumbleOneTimer.start();
    rumbleTwoTimer.start();
    frontSensorTolTimer.reset();
    frontSensorTolTimer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    boolean stopElevator = loader.getUpSensor();
    boolean stopIndexer = loader.getUpSensor() && elevator.getMidSensor();
    if(stopIndexer) { elevator.setElevatorFull(true); }

    intake.setIntake(true);
    loader.runLoaderWheel(stopIndexer? 0.0 :Constants.Intake.loaderHoldSpeed);
    intake.rollersSpeed(stopIndexer? 0.0 :Constants.Intake.rollerSpeed);
    elevator.runElevator(stopElevator? 0.0 : Constants.Indexer.indexerSpeed);
    conveyor.runConveyor(stopIndexer? 0.0 : Constants.Indexer.indexerSpeed);

    //rumble
    boolean rumbleOneTripped = loader.getUpSensor();
    boolean rumbleTwoTripped = loader.getUpSensor() && elevator.getMidSensor();
    if (rumbleOneTripped && !rumbleLastOneTripped) {
      rumbleOneActive = true;
      rumbleOneTimer.reset();
    }
    if (rumbleTwoTripped && !rumbleLastTwoTripped) {
      rumbleTwoActive = true;
      rumbleTwoTimer.reset();
    }
    if (rumbleOneActive && !rumbleOneTimer.hasElapsed(Constants.rumbleDuration)
      || (rumbleTwoActive && !rumbleTwoTimer.hasElapsed(Constants.rumbleDuration))) {
        rumble.accept(Constants.rumblePercent);
    } else {
        rumble.accept(0.0);
    }

    rumbleLastOneTripped = rumbleOneTripped;
    rumbleLastTwoTripped = rumbleTwoTripped;

    //ball Counte
    boolean frontSensorTripped = conveyor.getFrontSensor();
    if(!frontSensorTripped && frontSensorTrippedLast) {
      int increment = (RobotContainer.ballCount == lastCount + 1)? 0 : 1;
      RobotContainer.ballCount+= increment;
      frontSensorTolTimer.reset();
    }
    frontSensorTrippedLast = frontSensorTripped;
    if (frontSensorTolTimer.hasElapsed(0.1)) {
      lastCount = RobotContainer.ballCount;
    }
    SmartDashboard.putNumber("lastCount", lastCount);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntake(false);
    intake.rollersSpeed(0);
    elevator.runElevator(0);
    conveyor.runConveyor(0);
    loader.runLoaderWheel(0);
    rumbleOneTimer.stop();
    rumbleTwoTimer.stop();
    rumble.accept(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
