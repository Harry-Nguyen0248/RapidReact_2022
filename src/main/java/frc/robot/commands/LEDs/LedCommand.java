/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.LEDs;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer.Elevator;
import frc.robot.subsystems.LEDsSub.LEDs;
import frc.robot.subsystems.ShooterSub.Shooter;
import frc.robot.util.LedValues;
import frc.robot.util.LedValues.FixedPalettePattern;

/**
 * A command to automatically set LED colors quickly and easily
 * Relies on {@link LEDTrigger}s to check when each LED pattern should be enabled
 */
public class LedCommand extends CommandBase {
  /**
   * A class to handle custom triggers for LED control
   */
  private class LEDTrigger implements Comparable<LEDTrigger> {
    private BooleanSupplier trigger;
    private double mode;
    private int priority;

    /**
    * A class to handle custom triggers for LED control
    * @param trigger A boolean supplier that, when active, will enable this trigger
    * @param mode The PWM value to send to the LED controller when this trigger is active
    * @param priority Integer value to determine which trigger should be prioritized, lower values are higher priority
    */
    public LEDTrigger(BooleanSupplier trigger, double mode, int priority) {
      this.trigger = trigger;
      this.mode = mode;
      this.priority = priority;
    }

    @Override
    public int compareTo(LEDTrigger other) {
      return this.priority - other.priority;
      // if(this.priority == other.priority)       { return 0; }
      // else if (this.priority > other.priority)  { return 1; }
      // else                                      { return -1; }
    }

    public boolean getTrigger() { return trigger.getAsBoolean(); }
    public double getMode() { return mode; } 
  }

  private LEDs leds;
  private List<LEDTrigger> triggers = new ArrayList<>();

  /**
   * Create a new {@code LedCommand} to control a specific {@link LEDs} subsystem
   * @param leds The {@link LEDs} subsystem that interfaces with the hardware on the robot
   */
  public LedCommand(LEDs leds) {
    this.leds = leds;
    addRequirements(leds);
  }

  /**
   * Add a {@link LEDTrigger} for the LED command to keep track of
   * @param trigger The {@link BooleanSupplier} that determines when the trigger is active
   * @param mode The PWM value to send to the LED controller when this trigger is active
   * @param priority The priority of this trigger. Since only one trigger can be active at a time, lowest priority value will dominate
   */
  public void addTrigger(BooleanSupplier trigger, double mode, int priority) {
    triggers.add(new LEDTrigger(trigger, mode, priority));
    triggers.sort(null);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    for (LEDTrigger trigger : triggers) {
      if(trigger.getTrigger()) {
        leds.set(trigger.getMode());
        break;
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }
}
