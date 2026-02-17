// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;

public class ToggleClimberServoCommand extends Command {
  /**
   * Creates a new ToggleClimberServoCommand that toggles both servos simultaneously between
   * deployed (1.0) and retracted (0.0) positions based on current position.
   */
  private final ClimberSubsystem climberSubsystem;

  /**
   * Creates a new ToggleClimberServoCommand.
   *
   * @param climberSubsystem The climber subsystem to control
   */
  public ToggleClimberServoCommand(ClimberSubsystem climberSubsystem) {
    this.climberSubsystem = climberSubsystem;
    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Read current left servo position and calculate new position
    // (both servos should be in sync, so we only need to check one)
    double currentPosition = climberSubsystem.getLeftServoPosition();
    double newPosition = (currentPosition > 0.5) ? 0.0 : 1.0;
    // Set both servos to the toggled position simultaneously
    climberSubsystem.setServoPositions(newPosition);
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
    return true;
  }
}
