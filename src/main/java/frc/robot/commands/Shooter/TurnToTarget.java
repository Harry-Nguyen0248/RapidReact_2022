package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSub.DriveTrain;
import frc.robot.subsystems.VisionSub.Vision;

public class TurnToTarget extends CommandBase {
    private Vision ll; 
    private DriveTrain dt;
    private boolean isOnTarget = false;

    public TurnToTarget(Vision ll, DriveTrain dt) {
        this.ll = ll; 
        this.dt = dt; 

        addRequirements(dt); 
      }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() { 
        //ll.setLLLed(true);
        dt.brakeOn();
        isOnTarget = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() { 
        if (ll.isLedOn() && ll.isTargetFound()) {
            double amountAdjust = ll.getX() / Constants.turnToTargetSpeedMod; // PID is too hard
            if (Math.abs(ll.getX()) > Constants.turnToTargetTolerance) {
                dt.arcadeDriveDirect(0, amountAdjust /*+ Math.copySign(Constants.minMotorPower, amountAdjust) /**/); 
            } else {
                isOnTarget = true;
            }
            SmartDashboard.putNumber("adjustment", amountAdjust);

        }
        SmartDashboard.putBoolean("isOnTarget", isOnTarget);
}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //ll.setLLLed(false);
        dt.brakeOff();
        dt.stop();
        isOnTarget = false; // reset
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isOnTarget;
    }

    public boolean isOnTarget() { return isOnTarget; }
}