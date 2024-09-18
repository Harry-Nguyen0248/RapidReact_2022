// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClimberSub;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Climber extends SubsystemBase {

  private Solenoid latchPiston; // holds down climber arm before extension
  private Solenoid climbingPiston; // pulls down climber arm after extension

  /** Creates a new Climber. */
  public Climber(int extendingPistonID, int climbingPistonID/*int climbingPistonIDf, int climbingPistonIDr*/) {
    // extendingPiston lengthens hooks to grab onto the bar, climbingPiston raises base of robot once on bar
    latchPiston = new Solenoid(PneumaticsModuleType.CTREPCM, extendingPistonID);
    // climbingPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, climbingPistonIDf, climbingPistonIDr);
    this.climbingPiston = new Solenoid(PneumaticsModuleType.CTREPCM, climbingPistonID);
  }
  
  /**
   * Reset the climbing piston to be in the neutral position.
   * Do this before the match starts (or at least before trying to climb) so it is easily put into position by the CF spring.
   */
  // public void resetClimbingPiston() {
  //   climbingPiston.set(Value.kOff);
  // }

  //sets piston to extend or retract
  public void extendClimberArms() {
    latchPiston.set(true);
  }

  public void setLiftPistonOn() {
    climbingPiston.set(true);
  }

  public void liftRobot() {
    climbingPiston.set(false);
  }
/*
  public void retractClimberArms() {
    extendingClimberPiston.set(false);
  }
*/

  // public void liftRobotOffGround() { 
  //   // Retracting piston pulls robot up (?)
  //   climbingPiston.set(Value.kForward);
  // }

  /*
  public void lowerRobot() { 
    // Retracting piston pulls robot up (?)
    climbingPiston.set(false);
  }
*/

  //check if piston is extended or retracted

  public boolean getExtendingPistonState(){ 
    return latchPiston.get();
  }
  
  public boolean getClimbingPistonState(){ 
    return climbingPiston.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
