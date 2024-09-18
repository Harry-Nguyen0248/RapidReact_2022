// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ShooterSub;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import frc.robot.Constants;

public class ShooterConfig {

  public static void shooterConfig(CANSparkMax lower, CANSparkMax upper) {
    upper.restoreFactoryDefaults();
    lower.restoreFactoryDefaults();

    lower.setSmartCurrentLimit(30);
    upper.setSmartCurrentLimit(30);

    lower.enableVoltageCompensation(12.0);
    lower.getVoltageCompensationNominalVoltage();
    upper.enableVoltageCompensation(12.0);
    upper.getVoltageCompensationNominalVoltage();

    lower.setClosedLoopRampRate(Constants.Shooter.flyWheelClosedLoopRampRate);
    upper.setClosedLoopRampRate(Constants.Shooter.flyWheelClosedLoopRampRate);

    upper.setInverted(Constants.Shooter.upperFlyWheelInverted);  
    lower.setInverted(Constants.Shooter.lowerFlyWheelInverted);

    upper.setIdleMode(IdleMode.kCoast);
    lower.setIdleMode(IdleMode.kCoast);

    // may not be best idea
    // upper.burnFlash();
    // lower.burnFlash();
  }

  public static void turretConfig(CANSparkMax turret) {
    turret.restoreFactoryDefaults();
    turret.setIdleMode(IdleMode.kBrake);

    turret.enableSoftLimit(SoftLimitDirection.kForward, true);
    turret.enableSoftLimit(SoftLimitDirection.kReverse, true);
    turret.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.Turret.softLimitForward);
    turret.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.Turret.softLimitBackward);

    turret.burnFlash();
  }
}
