// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;

public class GoToZeroCommand extends Command {
  /** Creates a new GoToZeroCommand that zeros the climber to bottom using stall detection. */
  private final ClimberSubsystem climberSubsystem;

  /**
   * Creates a new GoToZeroCommand. This command zeros the climber by driving downward until stall
   * current is detected (indicating the climber has hit the bottom).
   *
   * @param climberSubsystem The climber subsystem to control
   */
  public GoToZeroCommand(ClimberSubsystem climberSubsystem) {
    this.climberSubsystem = climberSubsystem;
    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climberSubsystem.zeroClimberToBottom();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climberSubsystem.isAtZero();
  }
}
