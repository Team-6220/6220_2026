// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

/**
 * Command to run the shooter at a target velocity.
 * You should consider using the more terse Command factories API instead 
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class ShooterCommand extends Command {
  private final ShooterSubsystem m_shooter;
  private final double m_targetRPM;
  private final boolean m_waitForSpeed;

  /**
   * Creates a new ShooterCommand that runs until interrupted.
   * @param shooter The shooter subsystem
   * @param targetRPM Target velocity in RPM
   */
  public ShooterCommand(ShooterSubsystem shooter, double targetRPM) {
    this(shooter, targetRPM, false);
  }

  /**
   * Creates a new ShooterCommand.
   * @param shooter The shooter subsystem
   * @param targetRPM Target velocity in RPM
   * @param waitForSpeed If true, command ends when at speed; if false, runs until interrupted
   */
  public ShooterCommand(ShooterSubsystem shooter, double targetRPM, boolean waitForSpeed) {
    m_shooter = shooter;
    m_targetRPM = targetRPM;
    m_waitForSpeed = waitForSpeed;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setTargetRPM(m_targetRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Command continuously maintains target velocity
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Keep shooter running even when command ends
    // If you want to stop on end, uncomment the line below
    // m_shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // If waitForSpeed is true, finish when at speed
    // Otherwise, run until interrupted
    return m_waitForSpeed && m_shooter.isAtSpeed();
  }

  /**
   * Nested command to stop the shooter.
   */
  public static class Stop extends Command {
    private final ShooterSubsystem m_shooter;

    /** Creates a new StopShooterCommand. */
    public Stop(ShooterSubsystem shooter) {
      m_shooter = shooter;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      m_shooter.stop();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return true; // Instant command
    }
  }
}