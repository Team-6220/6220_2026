// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber.ClimberSubsystem;

public class ClimberCommand extends Command {
  /** Creates a new ClimberCommand with motor control via Xbox controller. */
  private final ClimberSubsystem climberSubsystem;
  private final CommandXboxController controller;
  
  public ClimberCommand(ClimberSubsystem climberSubsystem, CommandXboxController controller) {
    this.climberSubsystem = climberSubsystem;
    this.controller = controller;
    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Left stick Y-axis controls left climber motor
    double leftYValue = -controller.getLeftY(); // Negative because forward is negative on Y-axis
    
    // Apply deadband and control left motor
    if (Math.abs(leftYValue) > 0.2) {
      climberSubsystem.simpleDriveLeft(leftYValue);
    } else {
      climberSubsystem.simpleDriveLeft(0);
    }
    
    // TODO: Add right motor control when installed on full robot
    // Right stick Y-axis would control right climber motor
    // double rightYValue = -controller.getRightY();
    // if (Math.abs(rightYValue) > 0.2) {
    //   climberSubsystem.simpleDriveRight(rightYValue);
    // } else {
    //   climberSubsystem.simpleDriveRight(0);
    // }
    
    // A button sets servo to configured angle
    if (controller.a().getAsBoolean()) {
      // Convert angle (0-180 degrees) to servo position (0.0-1.0)
      double servoPosition = ClimberConstants.servoAngleDegrees / 180.0;
      climberSubsystem.setServoPosition(servoPosition);
    }
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