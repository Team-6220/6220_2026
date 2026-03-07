// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

/**
 * Runs all shooter motors at percent output while the command is active. Stops when the command
 * ends (e.g., button released). Swap to runAtTargetVelocity() once PID is tuned.
 */
public class ShooterCommand extends Command {

  private final ShooterSubsystem m_shooter;

  public ShooterCommand(ShooterSubsystem shooter) {
    m_shooter = shooter;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Use percent output for basic spin testing (change 0.5 to go faster/slower)
    m_shooter.setPercentOutput(0.5);
    // Swap to this once PID is tuned:
    // m_shooter.runAtTargetVelocity();
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
