// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.AnglerSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PassToAlliance extends Command {
  /** Creates a new PassToAlliance. */
  AnglerSubsystem m_angler;

  ShooterSubsystem m_shoot;

  public PassToAlliance(AnglerSubsystem m_angler, ShooterSubsystem m_shoot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_angler = m_angler;
    this.m_shoot = m_shoot;
    addRequirements(m_angler, m_shoot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_angler.setAngle(35);
    m_shoot.runAtTargetVelocity(2200);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_angler.stop();
    m_shoot.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
