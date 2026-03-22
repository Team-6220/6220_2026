// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignHub;
import frc.robot.commands.HashShoot;
import frc.robot.subsystems.Drive.Swerve;
import frc.robot.subsystems.Shooter.AnglerSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;



// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BasicAuto extends SequentialCommandGroup {

    private AnglerSubsystem angler;
    private ShooterSubsystem shooter;
    private Swerve s_swerve;
  /** Creates a new StrightAuto. */
  public BasicAuto(Swerve aswerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    angler = new AnglerSubsystem();
    shooter = new ShooterSubsystem();
    addRequirements(s_swerve);
    addCommands(
        // AutoBuilder.pathfindToPose(new Pose2d(-1, 0, new Rotation2d()),
        // AutoConstants.pathConstraints),
        // new InstantCommand(() -> s_swerve.setPose(AutoConstants.startPosesBlue[0])),
        new PrintCommand("starting basic auto"),
        new RunCommand(() -> s_swerve.drive(new Translation2d(1, 0), 0, true, false))
            .withTimeout(5),
        new RunCommand(() -> s_swerve.drive(new Translation2d(0, 0), 0, true, false)).withTimeout(0.2),
        new AlignHub(s_swerve).withTimeout(2),
        new HashShoot(angler, shooter).withTimeout(2),
        new PrintCommand("done")
        );
  }
}