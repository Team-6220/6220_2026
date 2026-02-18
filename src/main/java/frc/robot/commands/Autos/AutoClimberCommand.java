// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GoToZeroCommand;
import frc.robot.commands.SetClimberServoCommand;
import frc.robot.subsystems.Climber.ClimberSubsystem;

/**
 * Auto climber sequence: actuates servo, zeros climber to bottom using stall detection, then stops
 * servo. This command is designed for use in autonomous routines.
 */
public class AutoClimberCommand extends SequentialCommandGroup {
  /**
   * Creates a new AutoClimberCommand.
   *
   * @param climberSubsystem The climber subsystem to control
   */
  public AutoClimberCommand(ClimberSubsystem climberSubsystem) {
    addCommands(
        // Step 1: Actuate the servo to deployed position
        new SetClimberServoCommand(climberSubsystem, 1.0),
        // Step 2: Zero climber to bottom using stall detection
        new GoToZeroCommand(climberSubsystem),
        // Step 3: Deactuate the servo
        new SetClimberServoCommand(climberSubsystem, 0.0));
  }
}
