// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import frc.robot.Constants;

public class Configuration {

    //Intake

    //indexer


    //shooter
    public static void turretConfig(CANSparkMax turret) {
        // set to defaults
        turret.restoreFactoryDefaults(); 
        // set to brake when idle
        turret.setIdleMode(IdleMode.kBrake);
        // set software limits so the turret avoids 
        turret.setSoftLimit(SoftLimitDirection.kForward, Constants.Shooter.turretSoftLimitForward);
        turret.setSoftLimit(SoftLimitDirection.kReverse, Constants.Shooter.turretSoftLimitReverse);

        turret.enableSoftLimit(SoftLimitDirection.kForward, false);
        turret.enableSoftLimit(SoftLimitDirection.kReverse, false);

        // configure hardware limit switches
        turret.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
      }

}
