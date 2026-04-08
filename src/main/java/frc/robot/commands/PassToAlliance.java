// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake.BeltSubsystem;
import frc.robot.subsystems.Shooter.AnglerSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PassToAlliance extends Command {
  /** Creates a new PassToAlliance. */
  AnglerSubsystem m_angler;

  ShooterSubsystem m_shoot;

  BeltSubsystem m_belt;

  CommandXboxController controller;

  public PassToAlliance(
      AnglerSubsystem m_angler,
      ShooterSubsystem m_shoot,
      BeltSubsystem m_belt,
      CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_angler = m_angler;
    this.m_shoot = m_shoot;
    this.m_belt = m_belt;
    this.controller = controller;
    addRequirements(m_angler, m_shoot, m_belt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_angler.setAngle(32);
    m_shoot.runAtTargetVelocity(2200, controller);
    m_belt.simpleDrive(-0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_angler.stop();
    m_shoot.stop();
    m_belt.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
