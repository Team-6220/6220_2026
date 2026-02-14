// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;

public class SetClimberServoCommand extends Command {
  /** Creates a new SetClimberServoCommand that sets the servo to the configured angle. */
  private final ClimberSubsystem climberSubsystem;

  private final double servoPosition;

  /**
   * Creates a new SetClimberServoCommand.
   *
   * @param climberSubsystem The climber subsystem to control
   * @param servoPosition The servo position (0.0 to 1.0)
   */
  public SetClimberServoCommand(ClimberSubsystem climberSubsystem, double servoPosition) {
    this.climberSubsystem = climberSubsystem;
    this.servoPosition = servoPosition;
    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Convert angle (0-180 degrees) to servo position (0.0-1.0)
    double position = servoPosition;
    climberSubsystem.setServoPosition(position);
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
