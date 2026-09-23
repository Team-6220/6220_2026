// I'M JUST TRYING TO DRIVE HERE

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.SamAuto;

// TODO: AUTO - PathPlanner doesn't support WPILib 2027 alpha 7 yet. Restore these imports
// (and the path-following body below) once it does, or replace with a non-PathPlanner follower.
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.util.FileVersionException;
// import org.wpilib.math.geometry.Pose2d;
// import org.wpilib.math.geometry.Rotation2d;
// import java.io.IOException;
// import org.json.simple.parser.ParseException;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.SequentialCommandGroup;
import frc.robot.subsystems.Drive.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SamAutoV1 extends SequentialCommandGroup {
  /** Creates a new SamAutoV1. */
  public SamAutoV1(Swerve swerve) {
    addRequirements(swerve);

    // TODO: AUTO - PathPlanner path following ("toptostation") disabled until PathPlanner
    // supports WPILib 2027 alpha 7.
    // PathPlannerPath path;
    // try {
    //   path = PathPlannerPath.fromPathFile("toptostation");
    // } catch (IOException | ParseException | FileVersionException e) {
    //   throw new RuntimeException("Failed to load toptostation.path", e);
    // }
    //
    // Pose2d startPose =
    //     path.getStartingHolonomicPose().orElse(new Pose2d(2.27, 7.0, new Rotation2d()));
    //
    // addCommands(
    //     new InstantCommand(() -> swerve.setPose(startPose)),
    //     AutoBuilder.followPath(path),
    //     new InstantCommand(() -> System.out.println("Done with auto!")));

    addCommands(
        new InstantCommand(
            () -> System.out.println("SamAutoV1 disabled: PathPlanner not available (TODO: AUTO)")));
  }
}
