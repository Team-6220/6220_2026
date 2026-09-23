package frc.robot.commands;

import static org.wpilib.units.Units.Radians;

import frc.robot.AutoConstants;
import frc.robot.subsystems.Drive.Swerve;
import org.wpilib.command2.Command;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;

/**
 * Drives the robot to a field pose in a straight line using PID on X, Y, and heading.
 *
 * <p>Simple stand-in for PathPlanner's pathfindToPose while PathPlanner doesn't support WPILib 2027
 * alpha 7. X and Y each use a ProfiledPIDController so the robot accelerates and decelerates within
 * AutoConstants' translation limits. Heading uses Swerve's existing turn controller. Gains are read
 * from the AutoConstants tunables when the command starts, so dashboard edits apply on the next
 * run.
 */
public class DriveToPose extends Command {
  /** How close (meters) X and Y must each be to the target to finish. */
  private static final double POSITION_TOLERANCE_METERS = 0.05;

  private final Swerve s_Swerve;
  private final Pose2d targetPose;

  private ProfiledPIDController xController;
  private ProfiledPIDController yController;

  /**
   * @param s_Swerve the swerve subsystem
   * @param targetPose target pose in field coordinates (same frame as {@link Swerve#getPose()})
   */
  public DriveToPose(Swerve s_Swerve, Pose2d targetPose) {
    this.s_Swerve = s_Swerve;
    this.targetPose = targetPose;
    addRequirements(s_Swerve);
  }

  @Override
  public void initialize() {
    TrapezoidProfile.Constraints constraints =
        new TrapezoidProfile.Constraints(
            AutoConstants.translationMaxVel(), AutoConstants.translationMaxAccel());
    xController = createTranslationController(constraints);
    yController = createTranslationController(constraints);

    // Start the profiles from where the robot is and how fast it's already moving, so chaining
    // several DriveToPose commands doesn't cause a jerk at each waypoint.
    Pose2d pose = s_Swerve.getPose();
    ChassisVelocities fieldVelocities =
        s_Swerve.getRobotRelativeSpeeds().toFieldRelative(pose.getRotation());
    xController.reset(pose.getX(), fieldVelocities.vx);
    yController.reset(pose.getY(), fieldVelocities.vy);
    xController.setGoal(targetPose.getX());
    yController.setGoal(targetPose.getY());

    s_Swerve.resetTurnController();
    s_Swerve.setTurnControllerGoal(Radians.of(targetPose.getRotation().getRadians()));
  }

  @Override
  public void execute() {
    Pose2d pose = s_Swerve.getPose();

    // PID correction plus the profile's planned velocity (feedforward)
    double xSpeed = xController.calculate(pose.getX()) + xController.getSetpoint().velocity;
    double ySpeed = yController.calculate(pose.getY()) + yController.getSetpoint().velocity;
    double rotSpeed = s_Swerve.getTurnPidSpeed();

    s_Swerve.drive(new Translation2d(xSpeed, ySpeed), rotSpeed, true, false);
  }

  @Override
  public boolean isFinished() {
    double headingError =
        Math.abs(targetPose.getRotation().minus(s_Swerve.getHeading()).getRadians());
    return xController.atGoal()
        && yController.atGoal()
        && headingError < AutoConstants.angularToleranceRad();
  }

  @Override
  public void end(boolean interrupted) {
    s_Swerve.stopDriving();
  }

  private static ProfiledPIDController createTranslationController(
      TrapezoidProfile.Constraints constraints) {
    ProfiledPIDController controller =
        new ProfiledPIDController(
            AutoConstants.translationKPTN.get(),
            AutoConstants.translationKITN.get(),
            AutoConstants.translationKDTN.get(),
            constraints);
    controller.setTolerance(POSITION_TOLERANCE_METERS);
    return controller;
  }
}
