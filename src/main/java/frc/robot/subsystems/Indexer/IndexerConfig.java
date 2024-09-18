// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;

/** Add your docs here. */
public class IndexerConfig {
    public static void conveyorConfig(CANSparkMax conveyor)
    {
        // Resets conveyor settings to factory default. 
        conveyor.restoreFactoryDefaults();
        // Sets the conveyor to brake when idle. 
        conveyor.setIdleMode(IdleMode.kBrake);
        conveyor.setInverted(Constants.Indexer.conveyorInvert);
        conveyor.setSmartCurrentLimit(30);
        conveyor.enableVoltageCompensation(12.0);
    }

    public static void elevatorConfig(CANSparkMax elevator) {
        elevator.restoreFactoryDefaults();
        elevator.setIdleMode(IdleMode.kBrake);
        elevator.setInverted(Constants.Indexer.elevatorInvert);
        elevator.setSmartCurrentLimit(30);
        elevator.enableVoltageCompensation(12.0);
    }
    public static void feederConfig(CANSparkMax feeder) {
        feeder.restoreFactoryDefaults();
        feeder.setInverted(Constants.Shooter.feederInverted);
        feeder.setSmartCurrentLimit(20);
        feeder.enableVoltageCompensation(12.0);
    }
}
