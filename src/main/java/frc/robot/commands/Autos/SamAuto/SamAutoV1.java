//I'M JUST TRYING TO DRIVE HERE


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.SamAuto;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SamAutoV1 extends SequentialCommandGroup {
  /** Creates a new SamAutoV1. */
  public SamAutoV1(Swerve swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      AutoBuilder.pathfindToPose(
        new Pose2d(1,1,new Rotation2d(0)),
        new PathConstraints(MetersPerSecond.of(1),
        MetersPerSecondPerSecond.of(5),
        DegreesPerSecond.of(180),
      DegreesPerSecondPerSecond.of(360))),
      new InstantCommand(()-> System.out.println("Done with auto!"))
    );
  }
}
