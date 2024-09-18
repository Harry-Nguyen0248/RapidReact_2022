// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrainSub;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.Constants;

/** Add your docs here. */
public class DriveTrainConfig {

    public static final boolean isRightInverted = true;
    public static void configDriveTeleop(CANSparkMax leftM,CANSparkMax rightM, CANSparkMax leftS, CANSparkMax rightS) {
        // restoring defaults
        leftM.restoreFactoryDefaults();
        rightM.restoreFactoryDefaults();
        leftS.restoreFactoryDefaults();
        rightS.restoreFactoryDefaults();

        rightM.setInverted(isRightInverted);
        rightS.setInverted(isRightInverted);

        rightM.setIdleMode(IdleMode.kCoast);
        leftM.setIdleMode(IdleMode.kCoast);
        rightS.setIdleMode(IdleMode.kCoast);
        leftS.setIdleMode(IdleMode.kCoast);

        rightM.setSmartCurrentLimit(50);
        rightS.setSmartCurrentLimit(50);
        leftS.setSmartCurrentLimit(50);
        leftM.setSmartCurrentLimit(50);

        rightM.setOpenLoopRampRate(Constants.DriveTrain.driveTrainOpenLoopRampRate);
        leftM.setOpenLoopRampRate(Constants.DriveTrain.driveTrainOpenLoopRampRate);

        // setting slaves to follow
        leftS.follow(leftM);
        rightS.follow(rightM);

        System.out.println("<----------------teleop----------->");
    }

    public static void configDriveAuto(CANSparkMax leftM,CANSparkMax rightM, CANSparkMax leftS, CANSparkMax rightS, Gyro gyro) {
        // restoring defaults
        leftM.restoreFactoryDefaults();
        rightM.restoreFactoryDefaults();
        leftS.restoreFactoryDefaults();
        rightS.restoreFactoryDefaults();

        rightM.setInverted(isRightInverted);
        rightS.setInverted(isRightInverted);

        rightM.setIdleMode(IdleMode.kBrake);
        leftM.setIdleMode(IdleMode.kBrake);
        rightS.setIdleMode(IdleMode.kBrake);
        leftS.setIdleMode(IdleMode.kBrake);

        //setting slaves to follow
        leftS.follow(leftM);
        rightS.follow(rightM);

        rightM.setSmartCurrentLimit(50);
        rightS.setSmartCurrentLimit(50);
        leftS.setSmartCurrentLimit(50);
        leftM.setSmartCurrentLimit(50);

        gyro.calibrate();
        
        System.out.println("<----------------auto------------------------------->");
    }

    public static void driveDisabled(CANSparkMax leftM,CANSparkMax rightM, CANSparkMax leftS, CANSparkMax rightS) {
        configDriveTeleop(leftM, rightM, leftS, rightS);
        rightM.setIdleMode(IdleMode.kCoast);
        leftM.setIdleMode(IdleMode.kCoast);
        rightS.setIdleMode(IdleMode.kCoast);
        leftS.setIdleMode(IdleMode.kCoast);
    }
}
