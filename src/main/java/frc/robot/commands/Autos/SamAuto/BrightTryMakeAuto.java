package frc.robot.commands.Autos.SamAuto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import frc.robot.AutoConstants;
import frc.robot.commands.AlignAndMove;
import frc.robot.commands.ArmToPositionCommand;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.HashShootAuto;
import frc.robot.commands.TestRollerCommand;
import frc.robot.subsystems.Drive.Swerve;
import frc.robot.subsystems.Intake.ArmSubsystem;
import frc.robot.subsystems.Intake.BeltSubsystem;
import frc.robot.subsystems.Shooter.AnglerSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

import java.io.IOException;
import org.json.simple.parser.ParseException;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.WaitCommand;
import org.wpilib.command2.button.CommandXboxController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import static org.wpilib.units.Units.MetersPerSecond;

public class BrightTryMakeAuto extends SequentialCommandGroup{
      /** Starting pose (toptostation.path first anchor, idealStartingState rotation). */
  private static final Pose2d START_POSE = new Pose2d(3.43, 4.05, Rotation2d.fromDegrees(0));
  private final ArmSubsystem arm;
  private final CommandXboxController controller;
  /** Poses to drive to, in order (last one = toptostation.path end anchor + goalEndState). */
  private static final Pose2d[] ALIGNTOOUTPOST = {
        new Pose2d(1.7395, 5.946, Rotation2d.fromDegrees(180)),

    new Pose2d(1.176, 5.946, Rotation2d.fromDegrees(180)),
  };
  private static final Pose2d[] INTAKEWAYPOINTS = {
        new Pose2d(0.7, 5.946, Rotation2d.fromDegrees(180)),

  };
    private static final Pose2d[] SHOOTWAYPOINTS = {
        new Pose2d(2.1, 5.12, Rotation2d.fromDegrees(0)),
        new Pose2d(2.15, 3.98, Rotation2d.fromDegrees(0)),


  };

  /** Give up on any single waypoint after this long so auto can't hang. */
  private static final double WAYPOINT_TIMEOUT_SECONDS = 4.0;

  /** Creates a new SamAutoV2. */
  public BrightTryMakeAuto(Swerve swerve, ArmSubsystem arm,CommandXboxController controller, AnglerSubsystem angler, ShooterSubsystem shooter, BeltSubsystem belt) {
    this.arm = arm;
    this.controller = controller;
    addRequirements(swerve);

    addCommands(new InstantCommand(() -> swerve.setPose(START_POSE)));
    for (Pose2d waypoint : ALIGNTOOUTPOST) {
      addCommands(new DriveToPose(swerve, waypoint).withTimeout(WAYPOINT_TIMEOUT_SECONDS));
    };
    addCommands(new InstantCommand(()-> AutoConstants.translationMaxVelMpsTN.set(MetersPerSecond.of(0.5).in(MetersPerSecond))));
    addCommands(new TestRollerCommand(true)
      .alongWith(new ArmToPositionCommand(this.arm, -2)).withDeadline(
        new DriveToPose(swerve, INTAKEWAYPOINTS[0]).withTimeout(WAYPOINT_TIMEOUT_SECONDS).andThen(new WaitCommand(0.5))
      )
      );
          addCommands(new InstantCommand(()-> AutoConstants.translationMaxVelMpsTN.set(MetersPerSecond.of(5).in(MetersPerSecond))));

  for (Pose2d waypoint : SHOOTWAYPOINTS) {
      addCommands(new DriveToPose(swerve, waypoint).withTimeout(WAYPOINT_TIMEOUT_SECONDS));
    };
    addCommands(new AlignAndMove(swerve, this.controller, this.controller.rightBumper()).withTimeout(2),
        new HashShootAuto(angler, shooter, belt).withTimeout(4));


    addCommands(new InstantCommand(() -> System.out.println("Done with SamAutoV2!")));
  }

}
