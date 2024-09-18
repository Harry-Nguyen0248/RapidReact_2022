// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LoaderWheel extends SubsystemBase {
  private CANSparkMax loaderWheel;
  private DigitalInput upSensor;
  /** Creates a new LoaderWheel. */
  public LoaderWheel(int loaderWheelID, int upSensorID, MotorType type) {
    this.loaderWheel = new CANSparkMax(loaderWheelID, type);
    this.upSensor = new DigitalInput(upSensorID);
    IndexerConfig.feederConfig(loaderWheel);

  }

  public void runLoaderWheel(double speed) {
    loaderWheel.set(speed);
  }

  public boolean getUpSensor() {
    return upSensor.get();
  }





  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("upSensor", getUpSensor());
  }
}
