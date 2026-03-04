// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber.ClimberSubsystem;

/**
 * Auto climber sequence: toggles servo to deploy, zeros climber to bottom using stall detection,
 * then toggles servo to retract. The servo is initialized to retracted (0.0) on robot startup, so
 * the first toggle will always deploy it.
 */
public class AutoClimberCommand extends SequentialCommandGroup {
  /**
   * Creates a new AutoClimberCommand.
   *
   * @param climberSubsystem The climber subsystem to control
   */
  public AutoClimberCommand(ClimberSubsystem climberSubsystem) {
    addCommands(
        // Step 1: Toggle servo to deployed position (will always deploy since initialized to 0.0)
        new ToggleClimberServoCommand(climberSubsystem),
        // Step 2: Zero climber to bottom using stall detection
        new GoToZeroCommand(climberSubsystem),
        // Step 3: Toggle servo back to retracted position
        new ToggleClimberServoCommand(climberSubsystem));
  }
}
