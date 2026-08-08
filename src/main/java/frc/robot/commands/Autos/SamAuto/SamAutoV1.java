// I'M JUST TRYING TO DRIVE HERE

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.SamAuto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive.Swerve;
import java.io.IOException;
import org.json.simple.parser.ParseException;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SamAutoV1 extends SequentialCommandGroup {
  /** Creates a new SamAutoV1. */
  public SamAutoV1(Swerve swerve) {
    addRequirements(swerve);

    PathPlannerPath path;
    try {
      path = PathPlannerPath.fromPathFile("toptostation");
    } catch (IOException | ParseException | FileVersionException e) {
      throw new RuntimeException("Failed to load toptostation.path", e);
    }

    Pose2d startPose =
        path.getStartingHolonomicPose().orElse(new Pose2d(2.27, 7.0, new Rotation2d()));

    addCommands(
        new InstantCommand(() -> swerve.setPose(startPose)),
        AutoBuilder.followPath(path),
        new InstantCommand(() -> System.out.println("Done with auto!")));
  }
}
