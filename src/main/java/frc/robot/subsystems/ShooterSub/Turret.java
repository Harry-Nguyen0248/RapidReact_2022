// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ShooterSub;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TuningTable;

public class Turret extends SubsystemBase {
  private CANSparkMax turret;
  private SparkMaxPIDController pid;
  private double target;

  private TuningTable kP = new TuningTable("turret kP");
  private TuningTable kI = new TuningTable("turret kI");
  private TuningTable kD = new TuningTable("turret kD");
  private TuningTable kFF = new TuningTable("turret kFF");
  private TuningTable kIz = new TuningTable("turret kIz");

  public Turret(int turretID) {
    turret = new CANSparkMax(turretID, MotorType.kBrushless);
    pid = turret.getPIDController();
    ShooterConfig.turretConfig(turret);

    pid.setP(Constants.Turret.kP);
    pid.setI(Constants.Turret.kI);
    pid.setD(Constants.Turret.kD);
    pid.setFF(Constants.Turret.kFF);
    pid.setIZone(Constants.Turret.kIz);

    kP.setDefault(Constants.Turret.kP);
    kI.setDefault(Constants.Turret.kI);
    kD.setDefault(Constants.Turret.kD);
    kFF.setDefault(Constants.Turret.kFF);
    kIz.setDefault(Constants.Turret.kIz);
  }

  public void setTargetAngle(double angle) {
    target = angle;
    pid.setReference(angle, ControlType.kPosition);
  }

  public double getTargetAngle() { return target; }

  public void setSpeed(double speed) {
    turret.set(speed);
  }

  @Override
  public void periodic() {
    if(kP.hasChanged()) {pid.setP(kP.get());}
    if(kI.hasChanged()) {pid.setI(kI.get());}
    if(kD.hasChanged()) {pid.setD(kD.get());}
    if(kFF.hasChanged()) {pid.setFF(kFF.get());}
    if(kIz.hasChanged()) {pid.setIZone(kIz.get());}
  }
}
