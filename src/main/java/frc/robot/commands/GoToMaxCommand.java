// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;

public class GoToMaxCommand extends Command {
  /**
   * Creates a new GoToMaxCommand that maxes the climber to top using stall detection. Controls only
   * the left climber by default. SOFT LIMIT - not a hard stop.
   */
  private final ClimberSubsystem climberSubsystem;

  private final boolean controlLeft;
  private final boolean controlRight;

  /**
   * Creates a new GoToMaxCommand. This command maxes the climber by driving upward until stall
   * current is detected (indicating the climber has hit the top). Controls left climber only by
   * default.
   *
   * @param climberSubsystem The climber subsystem to control
   */
  public GoToMaxCommand(ClimberSubsystem climberSubsystem) {
    this(climberSubsystem, true, false); // Default to left climber only
  }

  /**
   * Creates a new GoToMaxCommand with explicit control over which climber(s) to max.
   *
   * @param climberSubsystem The climber subsystem to control
   * @param controlLeft Whether to control the left climber
   * @param controlRight Whether to control the right climber
   */
  public GoToMaxCommand(
      ClimberSubsystem climberSubsystem, boolean controlLeft, boolean controlRight) {
    this.climberSubsystem = climberSubsystem;
    this.controlLeft = controlLeft;
    this.controlRight = controlRight;
    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climberSubsystem.maxClimberToTop(controlLeft, controlRight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climberSubsystem.isAtTop(controlLeft, controlRight);
  }
}
