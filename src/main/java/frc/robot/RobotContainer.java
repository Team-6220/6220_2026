// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class RobotContainer {

  // Subsystems
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();

  // Controller
  private final CommandXboxController m_controller = new CommandXboxController(1);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Left bumper: hold to spin motors 9, 2, 31
    m_controller
        .leftBumper()
        .whileTrue(
            Commands.runEnd(
                () -> m_shooter.setBottomGroupPercent(0.2),
                () -> m_shooter.stopBottomGroup(),
                m_shooter));

    // Right bumper: hold to spin motors 41, 1
    m_controller
        .rightBumper()
        .whileTrue(
            Commands.runEnd(
                () -> m_shooter.setTopGroupPercent(0.2),
                () -> m_shooter.stopTopGroup(),
                m_shooter));

    // Alternative: right trigger hold (uncomment to use)
    // m_operatorController.rightTrigger(0.1).whileTrue(new ShooterCommand(m_shooter));

    // Alternative: A button toggle (uncomment to use)
    // m_operatorController.a().toggleOnTrue(new ShooterCommand(m_shooter));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured.");
  }
}
