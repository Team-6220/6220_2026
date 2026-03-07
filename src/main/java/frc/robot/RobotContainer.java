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

  // Test preset for A button (angle in degrees, RPM for top/bottom)
  private static final double PRESET_ANGLE_DEG = 45.0;
  private static final double PRESET_TOP_RPM = 4000.0;
  private static final double PRESET_BOTTOM_RPM = 2500.0;

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Left bumper: hold to spin bottom motors (9, 31, 2) at tunable RPM
    m_controller
        .leftBumper()
        .whileTrue(
            Commands.runEnd(
                () -> m_shooter.setBottomGroupVelocityRPS(m_shooter.getBottomTargetRPM() / 60.0),
                () -> m_shooter.stopBottomGroup()));

    // Right bumper: hold to spin top motors (41, 1) at tunable RPM
    m_controller
        .rightBumper()
        .whileTrue(
            Commands.runEnd(
                () -> m_shooter.setTopGroupVelocityRPS(m_shooter.getTopTargetRPM() / 60.0),
                () -> m_shooter.stopTopGroup()));

    // A button: auto angle + spin up shooter to preset, hold to maintain
    m_controller
        .a()
        .whileTrue(
            Commands.parallel(
                Commands.run(() -> m_angler.setAngle(PRESET_ANGLE_DEG), m_angler),
                Commands.runEnd(
                    () -> {
                      m_shooter.setTopGroupVelocityRPS(PRESET_TOP_RPM / 60.0);
                      m_shooter.setBottomGroupVelocityRPS(PRESET_BOTTOM_RPM / 60.0);
                    },
                    () -> m_shooter.stop())));

    // Left joystick Y: manual angler control (slow)
    m_angler.setDefaultCommand(
        Commands.run(
            () -> {
              double input = -m_controller.getLeftY();
              if (Math.abs(input) < ANGLER_DEADBAND) {
                m_angler.stop();
              } else {
                m_angler.setSpeed(input * ANGLER_MAX_SPEED);
              }
            },
            m_angler));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured.");
  }
}