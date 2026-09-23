// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.SamAuto;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.SequentialCommandGroup;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.Drive.Swerve;

/**
 * Same drive as SamAutoV1 (top start to station), but using PID drive-to-pose instead of
 * PathPlanner. Poses are the waypoints from deploy/pathplanner/paths/toptostation.path.
 *
 * <p>Drives a straight line between waypoints. Add more poses to WAYPOINTS to steer around things.
 * Poses are blue-alliance field coordinates and are not flipped for red.
 */
public class SamAutoV2 extends SequentialCommandGroup {
  /** Starting pose (toptostation.path first anchor, idealStartingState rotation). */
  private static final Pose2d START_POSE = new Pose2d(2.0, 7.0, Rotation2d.fromDegrees(0));

  /** Poses to drive to, in order (last one = toptostation.path end anchor + goalEndState). */
  private static final Pose2d[] WAYPOINTS = {
    new Pose2d(1.176, 5.946, Rotation2d.fromDegrees(0)),
  };

  /** Give up on any single waypoint after this long so auto can't hang. */
  private static final double WAYPOINT_TIMEOUT_SECONDS = 4.0;

  /** Creates a new SamAutoV2. */
  public SamAutoV2(Swerve swerve) {
    addRequirements(swerve);

    addCommands(new InstantCommand(() -> swerve.setPose(START_POSE)));
    for (Pose2d waypoint : WAYPOINTS) {
      addCommands(new DriveToPose(swerve, waypoint).withTimeout(WAYPOINT_TIMEOUT_SECONDS));
    }
    addCommands(new InstantCommand(() -> System.out.println("Done with SamAutoV2!")));
  }
}
