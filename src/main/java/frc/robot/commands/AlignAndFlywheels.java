// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drive.Swerve;
import frc.robot.subsystems.Intake.BeltSubsystem;
import frc.robot.subsystems.Shooter.AnglerSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignAndFlywheels extends ParallelCommandGroup {
  /** Creates a new AlignAndFlywheels. */
  Swerve s_swerve;

  CommandXboxController controller;
  AnglerSubsystem m_angler;
  ShooterSubsystem m_shooter;
  BeltSubsystem m_belt;

  public AlignAndFlywheels(
      Swerve s_swerve,
      CommandXboxController controller,
      AnglerSubsystem m_angler,
      ShooterSubsystem m_shooter,
      BeltSubsystem m_belt) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new AlignAndMove(s_swerve, controller, controller.rightBumper()),
        new HashShoot(m_angler, m_shooter, m_belt, controller));
  }
}
