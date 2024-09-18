/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.LEDsSub;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LedValues.*;

/**
 * Subsystem to control of REV Blinkin modules
 */
public class LEDs extends SubsystemBase {

  Spark leds;

  /**
   * Default constructor that uses RobotMap ports
   */
  public LEDs() {
    leds = new Spark(Constants.RobotMap.LEDs);
  }

  /**
   * Constructor that uses a custom port
   * @param port The PWN port where the controller is located
   */
  public LEDs(int port) {
    leds = new Spark(port);
  }

  /**
   * Set LEDs to a {@link FixedPalettePattern}
   * @param colour The value to set the LEDs to
   */
  public void set(FixedPalettePattern colour) {
    leds.set(colour.get());
  }

  /**
   * Set LEDs to a {@link Colour1Pattern}
   * @param colour The value to set the LEDs to
   */
  public void set(Colour1Pattern colour) {
    leds.set(colour.get());
  }
  
  /**
   * Set LEDs to a {@link Colour2Pattern}
   * @param colour The value to set the LEDs to
   */
  public void set(Colour2Pattern colour) {
    leds.set(colour.get());
  }

  /**
   * Set LEDs to a {@link Colour1And2Pattern}
   * @param colour The value to set the LEDs to
   */
  public void set(Colour1And2Pattern colour) {
    leds.set(colour.get());
  }

  /**
   * Set LEDs to a {@link SolidColour}
   * @param colour The value to set the LEDs to
   */
  public void set(SolidColour colour) {
    leds.set(colour.get());
  }

  /**
   * Set LEDs to a {@link double}
   * @param colour The value to set the LEDs to
   */
  public void set(double value) {
    leds.set(value);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // public void setMode (LedMode mode) {
  //   switch(mode) {
  //     case DISABLED_RED:
  //       set(FixedPalettePattern.STROBE__RED);
  //       break;
  //     case DISABLED_BLUE:
  //       set(FixedPalettePattern.STROBE__BLUE);
  //       break;
  //     case DISABLED_NEUTRAL:
  //       set(FixedPalettePattern.STROBE__WHITE);
  //       break;
  //     case AUTO:
  //       set(Colour1Pattern.HEARTBEAT_FAST);
  //       break;
  //     case INTAKE_DROPPING:
  //       set(SolidColour.AQUA);
  //       break;
  //     case CARGO_IN:
  //       set(SolidColour.RED);
  //       break;
  //     case ELEVATOR_FULL:
  //       set(SolidColour.GREEN);
  //       break;
  //     case ELEVATOR_HALF:
  //       set(SolidColour.YELLOW);
  //       break;
  //     case FLYWHEELS_READY:
  //       set(FixedPalettePattern.STROBE__GOLD);
  //       break;
  //     case SHOOT:
  //       set(Colour1And2Pattern.SPARKLE__COLOR_1_ON_COLOR_2);
  //       break;
  //     case CLIMBING:
  //       set(Colour2Pattern.BREATH_FAST);
  //       break;
  //     case TELEOP_DEFAULT:
  //       set(SolidColour.BLACK);
  //       break;
  //     default:
  //       set(SolidColour.BLACK);
  //       break;
  //   }
  // }
}
