// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class OI {
    XboxController driver = new XboxController (Constants.RobotMap.driverController);
    XboxController operator = new XboxController (Constants.RobotMap.operatorController);

    public XboxController getOperator() {
        return operator;
    }
    
    /** 
    @param deadband
    @param input
    @return
    */
    private double deadbandCompensation(double input, double deadband) {
        double slope = 1 / (1-deadband); // m = rise/run
        double offset = 1 - slope; // b = y - mx
        if (input < 0.0) {
            return Math.abs(input) > deadband? (-1 * (slope * Math.abs(input) + offset)) : 0.0;
        } else if (input > 0.0) {
            return Math.abs(input) > deadband? (slope * Math.abs(input) + offset): 0.0;
        } else {
            return 0.0;
        }
    }

    public double getDriveTurn() {
        double d = getLeftJoyStickX(driver);
        return d;
    }

    public double getDriveThrottle() {
        double d = getRightTrigger(driver) - getLeftTrigger(driver);
        return d;
    }

    public double getLeftJoyStickX(XboxController cont) {
        return deadbandCompensation(cont.getLeftX(), Constants.OI.deadbandJoystick);
    }

    public double getLeftJoyStickY(XboxController cont) {
        return deadbandCompensation(cont.getLeftY(), Constants.OI.deadbandJoystick);
    }

    public double getRightJoyStickX(XboxController cont) {
        return deadbandCompensation(cont.getRightX(), Constants.OI.deadbandJoystick);
    }

    public double getRightTrigger(XboxController cont) {
        return deadbandCompensation(cont.getRightTriggerAxis(), Constants.OI.deadbandTrigger);
    }

    public double getLeftTrigger(XboxController cont) {
        return deadbandCompensation(cont.getLeftTriggerAxis(), Constants.OI.deadbandTrigger);
    }

    public boolean dPadUpPressed(XboxController cont) {
        return (cont.getPOV() < 45 || cont.getPOV() > 315)
                && cont.getPOV() != -1;
    }

    public boolean dPadDownPressed(XboxController cont) {
        return (cont.getPOV() < 225 && cont.getPOV() > 135)
                && cont.getPOV() != -1;
    }

    public boolean dPadLeftPressed(XboxController cont) {
        return (cont.getPOV() < 315 && cont.getPOV() > 225)
                && cont.getPOV() != -1;
    }

    public boolean dPadRightPressed(XboxController cont) {
        return (cont.getPOV() < 135 && cont.getPOV() > 45)
                && cont.getPOV() != -1;
    }

    /*Driver Triggers Control*/
    public Trigger getKachow()                      { return new Trigger(driver::getStartButton); }
    public Trigger getTurnToTargetButton()          { return new Trigger(driver::getAButton); }
    public Trigger getSlowForwards()                { return new Trigger(() -> dPadUpPressed(driver)); }
    public Trigger getSlowBackwards()               { return new Trigger(() -> dPadDownPressed(driver)); }
    public Trigger getSlowRightTurn()               { return new Trigger(() -> dPadRightPressed(driver)); }
    public Trigger getSlowLeftTurn()                { return new Trigger(() -> dPadLeftPressed(driver)); }
    public Trigger getClimberArmsExtensionButton()  { return new Trigger(driver::getXButton); }
    public Trigger getClimberLiftRobotButton()      { return new Trigger(driver::getYButton); }
    public Trigger getClimbingArmExtendButton()     { return new Trigger(driver::getBButton); }

    /*Operator Triggers control */
    public Trigger getIntakeForwardsTrigger()       { return new Trigger(() -> operator.getLeftTriggerAxis() > 0.5); }
    public Trigger getFlyWheelsPreparedTrigger()    { return new Trigger(() -> operator.getRightTriggerAxis() > 0.5); }
    public Trigger getFeedShooterButton()           { return new Trigger(operator::getBackButton); }
    public Trigger getClimbMode()                   {return new Trigger(operator::getStartButton); }
    public Trigger getLaunchPadShootButton()        { return new Trigger(operator::getBButton); }
    public Trigger getUpCloseShootButton()          { return new Trigger(operator::getYButton); }
    public Trigger getStopFlyWheelButotn()          { return new Trigger(operator::getXButton); }
    public Trigger getManualIndexerForward()        { return new Trigger(operator::getLeftBumper); }
    public Trigger getManualIndexerBackward()       { return new Trigger(operator::getRightBumper); }
    public Trigger getBallCountResetTrigger()       { return new Trigger(() -> dPadDownPressed(operator)); }
    
    public void setOperatorRumble(double percent) { operator.setRumble(RumbleType.kRightRumble, percent); }

    public void setDriverRumble(double percent) { driver.setRumble(RumbleType.kRightRumble, percent); }

}