// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSub.DriveTrain;

public class RamseteAuto  {
  public static Command getAutonomousCommand(DriveTrain m_drive) {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(Constants.Ramsete.ksVolts,
                                Constants.Ramsete.kvVoltSecondsPerMeter,
                                Constants.Ramsete.kaVoltSecondsSquaredPerMeter),
      Constants.Ramsete.kDriveKinematics,
      10
    );

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
      Constants.Ramsete.kMaxSpeedMetersPerSecond,
      Constants.Ramsete.kMaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(Constants.Ramsete.kDriveKinematics)
      // Apply the voltage constraint
      .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(new Translation2d(1, 0), new Translation2d(2, 0)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(3, 0, new Rotation2d(0)),
      // Pass config
      config
    );

    RamseteCommand ramseteCommand = new RamseteCommand(
      exampleTrajectory,
      m_drive::getPose,
      new RamseteController(Constants.Ramsete.kRamseteB, Constants.Ramsete.kRamseteZeta),
      new SimpleMotorFeedforward(
          Constants.Ramsete.ksVolts,
          Constants.Ramsete.kvVoltSecondsPerMeter,
          Constants.Ramsete.kaVoltSecondsSquaredPerMeter),
      Constants.Ramsete.kDriveKinematics,
      m_drive::getWheelSpeeds,
      new PIDController(Constants.Ramsete.kPDriveVel, 0, 0),
      new PIDController(Constants.Ramsete.kPDriveVel, 0, 0),
      // RamseteCommand passes volts to the callback
      m_drive::tankDriveVolts,
      m_drive
    );

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_drive.tankDriveVolts(0, 0));
  }
}

  