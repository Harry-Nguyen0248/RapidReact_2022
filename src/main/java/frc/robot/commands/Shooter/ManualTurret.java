package frc.robot.commands.Shooter;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSub.Turret;

public class ManualTurret extends CommandBase {
    private Turret turret;
    private boolean left, right;

    public ManualTurret(Turret turret, boolean left, boolean right) {
        this.turret = turret;
        this.left = left;
        this.right = right;

        addRequirements(turret);
    }

    @Override
    public void execute() {
        if(left) { turret.setSpeed(0.1); }
        else if(right) { turret.setSpeed(-0.1); }
        else { turret.setSpeed(0); }
    }
}