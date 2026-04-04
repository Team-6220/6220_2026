// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlignAndMove;
import frc.robot.commands.HashShoot;
import frc.robot.subsystems.Drive.Swerve;
import frc.robot.subsystems.Intake.BeltSubsystem;
import frc.robot.subsystems.Shooter.AnglerSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BasicAutoBlue extends SequentialCommandGroup {

  private AnglerSubsystem m_angler;
  private ShooterSubsystem m_shooter;
  private BeltSubsystem m_belt;
  private Swerve s_swerve;
  private CommandXboxController controller;

  /** Creates a new StrightAuto. */
  public BasicAutoBlue(
      Swerve s_swerve, AnglerSubsystem m_angler, ShooterSubsystem m_shooter, BeltSubsystem m_belt, CommandXboxController controller) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.s_swerve = s_swerve;
    this.m_angler = m_angler;
    this.m_shooter = m_shooter;
    this.m_belt = m_belt;
    this.controller = controller;
    addRequirements(s_swerve, m_angler, m_belt, m_shooter);
    addCommands(
        // AutoBuilder.pathfindToPose(new Pose2d(-1, 0, new Rotation2d()),
        // AutoConstants.pathConstraints),
        // new InstantCommand(() -> s_swerve.setPose(AutoConstants.startPosesBlue[0])),
        new PrintCommand("starting basic auto"),
        new InstantCommand(() -> s_swerve.zeroHeading(), s_swerve),
        new RunCommand(() -> s_swerve.drive(new Translation2d(-1, 0), 0, true, false))
            .withTimeout(1.2),
        new RunCommand(() -> s_swerve.drive(new Translation2d(0, 0), 0, true, false))
            .withTimeout(0.2),
        new AlignAndMove(s_swerve, controller, controller.rightBumper()).withTimeout(2),
        new HashShoot(m_angler, m_shooter, m_belt).withTimeout(4),
        new PrintCommand("done"));
  }
}
