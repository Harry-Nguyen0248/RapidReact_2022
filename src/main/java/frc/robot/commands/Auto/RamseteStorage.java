package frc.robot.commands.Auto;

import java.io.IOException;
import java.nio.file.Path;

import org.w3c.dom.events.MouseEvent;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Shooter.AutoFeedShooter;
import frc.robot.commands.Shooter.AutoFlyWheelsPrepared;
import frc.robot.commands.Shooter.TurnToTargetTest;
import frc.robot.subsystems.DriveTrainSub.DriveTrain;
import frc.robot.subsystems.Indexer.Conveyor;
import frc.robot.subsystems.Indexer.Elevator;
import frc.robot.subsystems.Indexer.LoaderWheel;
import frc.robot.subsystems.IntakeSub.Intake;
import frc.robot.subsystems.ShooterSub.Hood;
import frc.robot.subsystems.ShooterSub.Shooter;
import frc.robot.subsystems.VisionSub.Vision;

public class RamseteStorage {
    private DriveTrain dt;
    private Intake intake;
    private Elevator elevator;
    private Conveyor conveyor;
    private LoaderWheel loader;
    private Shooter shooter;
    private Hood hood;
    private Vision ll;

    public RamseteStorage(DriveTrain dt, Intake intake, Elevator elevator, Conveyor conveyor, LoaderWheel loader, Shooter shooter, Hood hood, Vision ll) {
        this.dt = dt;
        this.intake = intake;
        this.elevator = elevator;
        this.conveyor = conveyor;
        this.loader = loader;
        this.shooter = shooter;
        this.hood = hood;
        this.ll = ll;
    }

    // util methods for cleaner code
    private Trajectory getTrajectoryFromFile(String filename) {
        Trajectory trajectory = new Trajectory();
        try {
            Path path = Filesystem.getDeployDirectory().toPath().resolve(filename);
            trajectory = TrajectoryUtil.fromPathweaverJson(path);
        } catch (IOException ex) {
            DriverStation.reportError("unable to open trajectory: " + filename, ex.getStackTrace());
        }
        return trajectory;
    }
    private RamseteCommand getRamseteCommandFromTrajectory(Trajectory trajectory) {
        return new RamseteCommand(trajectory, dt::getPose, 
        new RamseteController(Constants.Ramsete.kRamseteB, Constants.Ramsete.kRamseteZeta), 
        new SimpleMotorFeedforward(Constants.Ramsete.ksVolts,
                                    Constants.Ramsete.kvVoltSecondsPerMeter, 
                                    Constants.Ramsete.kaVoltSecondsSquaredPerMeter),
        Constants.Ramsete.kDriveKinematics, dt::getWheelSpeeds,
        new PIDController(0.5, 0, 0),
        new PIDController(0.5, 0, 0), 
        dt::tankDriveVolts, dt);
    }
    private RamseteCommand getRamseteCommandFromFile(String filename) {
        return getRamseteCommandFromTrajectory(getTrajectoryFromFile(filename));
    }

    public Command TestAuto() {
        Trajectory testAutoTraj = getTrajectoryFromFile("paths/DriveStraight.wpilib.json");
        RamseteCommand testAutoPath = getRamseteCommandFromFile("paths/DriveStraight.wpilib.json");
        dt.resetOdometry(testAutoTraj.getInitialPose());
        return testAutoPath.andThen(() -> dt.tankDriveVolts(0, 0));
    }
    
    public Command OneCargoAuto() {
        Trajectory t = getTrajectoryFromFile("paths/blue_taxi.wpilib.json");
        RamseteCommand one = getRamseteCommandFromFile("paths/blue_taxi.wpilib.json");

        SequentialCommandGroup movement = new SequentialCommandGroup(one.andThen(() -> dt.tankDriveVolts(0, 0)));
        ParallelDeadlineGroup moveAndIntake = new ParallelDeadlineGroup(new WaitCommand(4), movement, new RunIntake(intake, conveyor, elevator, loader, d -> System.out.println(d)));
        ParallelRaceGroup shoot = new ParallelRaceGroup(new AutoFlyWheelsPrepared(shooter, ll, hood, r -> System.out.println(r)), new AutoFeedShooter(loader, elevator, conveyor, shooter), new WaitCommand(8));
        SequentialCommandGroup moveIntakeShoot = new SequentialCommandGroup(moveAndIntake, new TurnToTargetTest(dt, ll), shoot);

        InstantCommand reset = new InstantCommand(() -> dt.resetOdometry(t.getInitialPose()));

        return reset.andThen(moveIntakeShoot);
    }
    public Command TwoCargoAuto() {
        Trajectory t = getTrajectoryFromFile("paths/onecargoauto/1cargo.wpilib.json");
        RamseteCommand one = getRamseteCommandFromFile("paths/onecargoauto/1cargo.wpilib.json");
        RamseteCommand two = getRamseteCommandFromFile("paths/twocargoauto/pickupcargo.wpilib.json");

        ParallelCommandGroup auto = new ParallelCommandGroup();
        //auto.addCommands(new RunIntake(forwards, intake, conveyor, elevator, loader, rumble);

        SequentialCommandGroup ballAuto = new SequentialCommandGroup();
        ballAuto.addCommands(one.andThen(() -> dt.tankDriveVolts(0, 0)));
        ballAuto.addCommands(two.andThen(() -> dt.tankDriveVolts(0, 0)));

        auto.addCommands(ballAuto);

        dt.resetOdometry(t.getInitialPose());

        return auto;
    }

    public Command ThreeCargoAuto() {
        return null;
    }

    public Command FourCargoAuto() {
        return null;
    }

    public Command FiveCargoAuto() {
        Trajectory one = getTrajectoryFromFile("paths/fivecargoauto/startToFirstCargo.wpilib.json");
        //startToFirstCargo
        RamseteCommand oneCommand = getRamseteCommandFromFile("paths/fivecargoauto/startToFirstCargo.wpilib.json");
        //firstCargoToFirstShot
        RamseteCommand twoCommand = getRamseteCommandFromFile("paths/fivecargoauto/firstCargoToFirstShot.wpilib.json");
        //firstShotToSecondCargo
        RamseteCommand thirdCommand = getRamseteCommandFromFile("paths/fivecargoauto/firstShotToSecondCargo.wpilib.json");
        //secondCargoToSecondShot
        RamseteCommand fourthCommand = getRamseteCommandFromFile("paths/fivecargoauto/secondCargoToSecondShot.wpilib.json");
        //secondShotToThirdCargo
        RamseteCommand fifthCommand = getRamseteCommandFromFile("paths/fivecargoauto/secondShotToTerminal.wpilib.json");
        //thirdCargoToThirdShot
        RamseteCommand sixthCommand = getRamseteCommandFromFile("paths/fivecargoauto/terminalToThirdShot.wpilib.json");

        SequentialCommandGroup BallAuto = new SequentialCommandGroup();
        BallAuto.addCommands(oneCommand.andThen(() -> dt.tankDriveVolts(0, 0)));
        BallAuto.addCommands(twoCommand.andThen(()-> dt.tankDriveVolts(0, 0)));
        BallAuto.addCommands(thirdCommand.andThen(() -> dt.tankDriveVolts(0, 0)));
        BallAuto.addCommands(fourthCommand.andThen(()-> dt.tankDriveVolts(0, 0)));
        BallAuto.addCommands(fifthCommand.andThen(() -> dt.tankDriveVolts(0, 0)));
        BallAuto.addCommands(sixthCommand.andThen(()-> dt.tankDriveVolts(0, 0)));

        dt.resetOdometry(one.getInitialPose());

        return BallAuto.andThen(() -> dt.tankDriveVolts(0, 0));
    }   
    
}
