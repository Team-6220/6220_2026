// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;

public class ClimberCommand extends Command {
  /** Creates a new ClimberCommand with configurable motor speed. */
  private final ClimberSubsystem climberSubsystem;

  private final double speed;

  /**
   * Creates a new ClimberCommand that drives the left climber at a specified speed.
   *
   * @param climberSubsystem The climber subsystem to control
   * @param speed The motor speed from -1.0 to 1.0
   */
  public ClimberCommand(ClimberSubsystem climberSubsystem, double speed) {
    this.climberSubsystem = climberSubsystem;
    this.speed = speed;
    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Drive left climber motor at configured speed
    climberSubsystem.setMotorSpeed(speed);

    // TODO: Add right motor control when installed on full robot
    // climberSubsystem.setMotorSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
