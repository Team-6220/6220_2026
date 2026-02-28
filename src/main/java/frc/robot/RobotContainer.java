// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class RobotContainer {

  // Subsystems
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();

  // Controllers
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Operator right bumper: hold to spin up shooter
    m_operatorController.rightBumper().whileTrue(new ShooterCommand(m_shooter));

    // Alternative: right trigger hold (uncomment to use)
    // m_operatorController.rightTrigger(0.1).whileTrue(new ShooterCommand(m_shooter));

    // Alternative: A button toggle (uncomment to use)
    // m_operatorController.a().toggleOnTrue(new ShooterCommand(m_shooter));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured.");
  }
}
