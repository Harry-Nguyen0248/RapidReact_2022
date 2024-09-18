// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {
  
  private CANSparkMax conveyor;
  private DigitalInput frontSensor;

  /** Creates a new Conveyor. */
  public Conveyor(int conveyorID, int frontSensorID, MotorType type) {
    conveyor = new CANSparkMax(conveyorID, type);
    frontSensor = new DigitalInput(frontSensorID);
    IndexerConfig.conveyorConfig(conveyor);
  }

  public void runConveyor(double speed) {
      conveyor.set(speed);
  }

  public boolean getFrontSensor() {
    return frontSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("frontSensor", getFrontSensor());
  }
}
