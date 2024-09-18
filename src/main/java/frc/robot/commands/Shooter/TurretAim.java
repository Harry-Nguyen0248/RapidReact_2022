package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSub.Turret;
import frc.robot.subsystems.VisionSub.Vision;

public class TurretAim extends CommandBase {
    private Turret turret;
    private Vision ll;

    private boolean isOnTarget;

    public TurretAim(Turret turret, Vision ll) {
        this.turret = turret;
        this.ll = ll;

        addRequirements(turret);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() { 
        //ll.setLLLed(true);
        isOnTarget = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() { 

        if (ll.isLedOn() && ll.isTargetFound()) {
            double amountAdjust = ll.getX() * Constants.turretAimSpeedMod; // PID is too hard
            if (Math.abs(ll.getX()) > Constants.turretAimTolerance) {
                turret.setSpeed(amountAdjust); 
            } else {
                isOnTarget = true;
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //ll.setLLLed(false);
        turret.setSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isOnTarget;
    }
}
