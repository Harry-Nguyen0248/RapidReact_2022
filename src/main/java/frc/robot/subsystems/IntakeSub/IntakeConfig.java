// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakeSub;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;

/** Add your docs here. */
public class IntakeConfig {
    public static void intakeConfig(CANSparkMax rollers) {
        rollers.restoreFactoryDefaults();
        rollers.setIdleMode(IdleMode.kBrake);
        rollers.setInverted(Constants.Intake.rollerInvert);
        rollers.setSmartCurrentLimit(30);
        rollers.enableVoltageCompensation(12.0);
    }
}
