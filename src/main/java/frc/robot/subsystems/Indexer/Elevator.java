// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private CANSparkMax elevator;
  private DigitalInput midSensor;
  private boolean isFull;
  /** Creates a new Elevator. */
  public Elevator(int elevatorID, int midSensorID, MotorType type) {
    elevator = new CANSparkMax(elevatorID, type);
    this.midSensor = new DigitalInput(midSensorID);
    IndexerConfig.elevatorConfig(elevator);
  }

  public boolean getMidSensor() {
    return midSensor.get();
}

  public void runElevator(double speed) {
    elevator.set(speed);
  }

  public void setElevatorFull(boolean isFull) {
    this.isFull = isFull;
  }

  public boolean getElevatorFullState() {
    return isFull;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("midSensor", getMidSensor());
  }
}
