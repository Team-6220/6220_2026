// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.BeltSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

/**
 * Runs all shooter motors at the tunable target velocity while the command is active. Stops when
 * the command ends (e.g., button released).
 */
public class ShooterTESTER extends Command {

  ShooterSubsystem m_shooter;
  BeltSubsystem m_belt;

  public ShooterTESTER(ShooterSubsystem shooter, BeltSubsystem m_belt) {
    m_shooter = shooter;
    this.m_belt = m_belt;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_shooter.runAtTargetVelocity();
    m_belt.simpleDrive(-0.3);
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
