// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ShooterSub;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
  private final Solenoid hood;

  private boolean raised = false;

  /** Creates a new Hood. */
  public Hood(PneumaticsModuleType type, int hoodID) {
    hood = new Solenoid(type, hoodID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /** Raise the rood if returns true */
  public void setRaised(boolean raised) {
      this.raised = raised;
      hood.set(raised);
  }

  /** Check the current hood's state */
  public boolean getState() {
    return raised;
  }


}
