package frc.robot.subsystems.IntakeSub;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private CANSparkMax rollers;
    private Solenoid piston;         // solenoid to control piston to raise/lower rollers

    public Intake(int rollersID, MotorType type, PneumaticsModuleType moduleType, int pistonID) {
        this.rollers = new CANSparkMax(rollersID, type);
        this.piston = new Solenoid(moduleType, pistonID);
        IntakeConfig.intakeConfig(rollers);
    }

    public void configIntake() {
        IntakeConfig.intakeConfig(rollers);
    }
    

    // raise/lower intake
    public void setIntake(boolean deploy) {
        piston.set(deploy);
    }

    public void toggleIntake() {
        setIntake(!getPistonPos());
        // SmartDashboard.putBoolean("Intake Pos", getPistonPos());
    }

    // get intake postition
    public boolean getPistonPos() { 
        return piston.get(); 
    }

    public void rollersSpeed(double speed) {
            rollers.set(speed);
    }
}