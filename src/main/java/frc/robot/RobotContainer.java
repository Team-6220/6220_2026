// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter.AnglerSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class RobotContainer {

  // Subsystems
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final AnglerSubsystem m_angler = new AnglerSubsystem();

  // Controller
  private final CommandXboxController m_controller = new CommandXboxController(1);

  // Max angler speed (keep it slow)
  private static final double ANGLER_MAX_SPEED = 0.15;
  private static final double ANGLER_DEADBAND = 0.1;

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Left bumper: hold to spin motors 9, 31
    m_controller
        .leftBumper()
        .whileTrue(
            Commands.runEnd(
                () -> m_shooter.setBottomGroupPercent(0.8), () -> m_shooter.stopBottomGroup()));

    // Right bumper: hold to spin motors 41, 1
    m_controller
        .rightBumper()
        .whileTrue(
            Commands.runEnd(
                () -> m_shooter.setTopGroupPercent(0.5), () -> m_shooter.stopTopGroup()));

    // Left joystick Y: control angler up/down (slow)
    m_angler.setDefaultCommand(
        Commands.run(
            () -> {
              double input = -m_controller.getLeftY(); // negative so up = up
              if (Math.abs(input) < ANGLER_DEADBAND) {
                m_angler.stop();
              } else {
                m_angler.setSpeed(input * ANGLER_MAX_SPEED);
              }
            },
            m_angler));

    // A button: hold to test angler motor
    m_controller
        .a()
        .whileTrue(Commands.runEnd(() -> m_angler.setSpeed(0.15), () -> m_angler.stop()));
    m_controller
        .b()
        .whileTrue(Commands.runEnd(() -> m_angler.setSpeed(-0.15), () -> m_angler.stop()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured.");
  }
}