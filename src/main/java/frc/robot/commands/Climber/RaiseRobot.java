package frc.robot.commands.Climber;

import frc.robot.subsystems.ClimberSub.Climber; 
import edu.wpi.first.wpilibj2.command.CommandBase;


public class RaiseRobot extends CommandBase {
    Climber climber;  
  
    /** Creates a new AimShoot. */
    public RaiseRobot(Climber climber) {
      this.climber = climber;
      addRequirements(climber);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        climber.liftRobot();
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  }

