package frc.robot.subsystems.ShooterSub;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TuningTable;

public class Shooter extends SubsystemBase {

    private final TuningTable kP_Upper = new TuningTable("P_Upper_Shooter");
    private final TuningTable kI_Upper = new TuningTable("I_Upper_Shooter");
    private final TuningTable kD_Upper = new TuningTable("D_Upper_Shooter");
    private final TuningTable kF_Upper = new TuningTable("F_Upper_Shooter");
    private final TuningTable kP_Lower = new TuningTable("P_Lower_Shooter");
    private final TuningTable kI_Lower = new TuningTable("I_Lower_Shooter");
    private final TuningTable kD_Lower = new TuningTable("D_Lower_Shooter");
    private final TuningTable kF_Lower = new TuningTable("F_Lower_Shooter");

    // private final TuningTable flyWheelRPMRatio = new TuningTable("flyWheelRPMRatio");
    // private final TuningTable flywheelSpeedAdjustment = new TuningTable("FW SPEED ADJUSTMENT: ");

    private CANSparkMax upperWheel;
    private CANSparkMax lowerWheel;

    //private RelativeEncoder turretEnc;
    private RelativeEncoder flyWheelEnc_Lower;
    private RelativeEncoder flyWheelEnc_Upper;
    private SparkMaxPIDController lowerWheelPID;
    private SparkMaxPIDController upperWheelPID;
    

    // private SimpleMotorFeedforward ff_lower;
    // private SimpleMotorFeedforward ff_upper;

    private double lowerVelocity, upperVelocity;

    private double maxUpperVel, minUpperVel, maxUpperAcc,
                maxLowerVel, minLowerVel, maxLowerAcc,
                upperAllowedErr, lowerAllowedErr;


   // private double velocity = 0;

    public Shooter(int upperWheelID, int lowerWheelID, MotorType type) {
        this.upperWheel = new CANSparkMax(upperWheelID, type);
        this.lowerWheel = new CANSparkMax(lowerWheelID, type);
        ShooterConfig.shooterConfig(lowerWheel, upperWheel);

        kP_Upper.setDefault(Constants.Shooter.kP_Upper);
        kI_Upper.setDefault(Constants.Shooter.kI_Upper);
        kD_Upper.setDefault(Constants.Shooter.kD_Upper);
        kF_Upper.setDefault(Constants.Shooter.kFF_Upper);

        kP_Lower.setDefault(Constants.Shooter.kP_Lower);
        kI_Lower.setDefault(Constants.Shooter.kI_Lower);
        kD_Lower.setDefault(Constants.Shooter.kD_Lower);
        kF_Lower.setDefault(Constants.Shooter.kFF_Lower);

        // flyWheelRPMRatio.setDefault(Constants.Shooter.flyWheelRPMRatio);
        // flywheelSpeedAdjustment.setDefault(0);

        this.lowerWheelPID = lowerWheel.getPIDController();
        this.upperWheelPID = upperWheel.getPIDController();

        int upperSlot = 0;
        int lowerSlot = 0;

        maxLowerVel = 5490;
        maxLowerAcc = 3000;

        lowerWheelPID.setSmartMotionMaxVelocity(maxLowerVel, lowerSlot);
        lowerWheelPID.setSmartMotionMaxAccel(maxLowerAcc, lowerSlot);
        lowerWheelPID.setSmartMotionMinOutputVelocity(minLowerVel, lowerSlot);
        lowerWheelPID.setSmartMotionAllowedClosedLoopError(lowerAllowedErr, lowerSlot);
        
        lowerWheelPID.setP(kP_Lower.get()); // 0.00015 (3 zeros, 1 one, 1 five)
        lowerWheelPID.setI(kI_Lower.get());
        lowerWheelPID.setD(kD_Lower.get()); //0.00012
        lowerWheelPID.setFF(kF_Lower.get()); //0.0001815 (3 zeros, 1 one, 1 eight, 1 one, 1 five)

        maxUpperVel = 5240;
        maxUpperAcc = 3000;

        upperWheelPID.setSmartMotionMaxVelocity(maxUpperVel, upperSlot);
        upperWheelPID.setSmartMotionMaxAccel(maxUpperAcc, upperSlot);
        upperWheelPID.setSmartMotionMinOutputVelocity(minUpperVel, upperSlot);
        upperWheelPID.setSmartMotionAllowedClosedLoopError(upperAllowedErr, upperSlot);

        upperWheelPID.setP(kP_Upper.get()); //0.00003 (4 zeros, 1 three)
        upperWheelPID.setI(kI_Upper.get());
        upperWheelPID.setD(kD_Upper.get()); // 0.0005 (3 zeros, 1 five)
        upperWheelPID.setFF(kF_Upper.get()); //0.0001807 (3 zeros, 1 one, 1 eight, 1 zero, 1 seven)


        // ff_lower = new SimpleMotorFeedforward(Constants.Shooter.kS_Lower, Constants.Shooter.kV_Lower);
        // ff_upper = new SimpleMotorFeedforward(Constants.Shooter.KS_Upper, Constants.Shooter.KV_Upper);

        this.flyWheelEnc_Lower = lowerWheel.getEncoder();
        this.flyWheelEnc_Upper = upperWheel.getEncoder();
    }

    public void setVelocity(double lowerVelocity, double upperVelocity) {
        // lowerWheelPID.setReference(lowerVelocity, ControlType.kSmartVelocity, 0, ff_lower.calculate(lowerVelocity), ArbFFUnits.kVoltage);

        // upperWheelPID.setReference(upperVelocity, ControlType.kSmartVelocity, 0, ff_upper.calculate(upperVelocity), ArbFFUnits.kVoltage);

        this.upperVelocity = upperVelocity;
        this.lowerVelocity = lowerVelocity;

        REVLibError lowerError =  lowerWheelPID.setReference(lowerVelocity, ControlType.kVelocity);
        if (lowerError != REVLibError.kOk) {
            SmartDashboard.putString("LOWER_SHOOTER_NOT_WORKING", lowerError.toString());
        }

        upperWheelPID.setReference(upperVelocity, ControlType.kVelocity);

    }

    public void runRPM(double rpm) {
        lowerWheel.set(rpm);
    }

    // handles targeting
    public void periodic() {
        // read PID coefficients from SmartDashboard
        // if (kP_Lower.hasChanged()) { lowerWheelPID.setP(kP_Lower.get());}
        // if (kI_Lower.hasChanged()) { lowerWheelPID.setI(kI_Lower.get());}
        // if (kD_Lower.hasChanged()) { lowerWheelPID.setD(kD_Lower.get());}

        // if (kP_Upper.hasChanged()) { upperWheelPID.setP(kP_Upper.get());}
        // if (kI_Upper.hasChanged()) { upperWheelPID.setI(kI_Upper.get());}
        // if (kD_Upper.hasChanged()) { upperWheelPID.setD(kD_Upper.get());}

        SmartDashboard.putNumber("currentLowerVel", getLowerCurrentRPM());//*100/Constants.Shooter.maxRPM)*//;
        SmartDashboard.putNumber("currentUpperVel", getUpperCurrentRPM());//**100/Constants.Shooter.maxRPM)*//;
        SmartDashboard.putNumber("desiredUpperVel", upperVelocity);
        SmartDashboard.putNumber("desiredLowerVel", lowerVelocity);
        // SmartDashboard.putNumber("velocity", this.velocity);

    }

    public void runLoweredHoodVelocity(double x) {
        this.lowerVelocity = 3763 + (-57.5) * x + 0.528 * Math.pow(x, 2); 
        this.upperVelocity = 3163 + (-57.5) * x + 0.528 * Math.pow(x, 2);
        setVelocity(this.lowerVelocity, this.upperVelocity);
    }

    public void runRaisedHoodVelocity(double x) {
        this.lowerVelocity = 6157 + (-445) * x + 19.4 * Math.pow(x, 2) + (-0.29) * Math.pow(x, 3);
        this.upperVelocity = 5057 + (-445) * x + 19.4 * Math.pow(x, 2) + (-0.29) * Math.pow(x, 3);
        setVelocity(this.lowerVelocity, this.upperVelocity);
    }

    public boolean flyWheelsReady() {
        return (this.lowerVelocity != Constants.Shooter.flyWheelIdleSpeed 
            || this.upperVelocity != Constants.Shooter.flyWheelIdleSpeed)
            && Math.abs(getLowerCurrentRPM() - this.lowerVelocity) <= Constants.Shooter.lowerToleranceRpm
            && Math.abs(getUpperCurrentRPM() - this.upperVelocity) <= Constants.Shooter.upperToleranceRpm;
    }

    public double getLowerCurrentRPM() {
        return flyWheelEnc_Lower.getVelocity();
    }

    public double getUpperCurrentRPM() {
        return flyWheelEnc_Upper.getVelocity();
    }
} 
