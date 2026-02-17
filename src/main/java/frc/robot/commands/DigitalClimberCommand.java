// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;

public class DigitalClimberCommand extends Command {
  /**
   * Creates a new DigitalClimberCommand that controls climber motors via digital buttons. Up
   * button uses stall detection to safely move to max. Down button uses stall detection to safely
   * move to zero. Left and right enable buttons must be held to allow their respective sides to
   * move. Reset button clears the AtTop and AtBottom flags.
   */
  private final ClimberSubsystem climberSubsystem;
  private final GenericHID buttonBoard;
  private final int upButton;
  private final int downButton;
  private final int resetButton;
  private final int leftEnableButton;
  private final int rightEnableButton;

  /**
   * Creates a new DigitalClimberCommand.
   *
   * @param climberSubsystem The climber subsystem to control
   * @param buttonBoard The button board controller
   * @param upButton Button number for moving climber up (uses stall detection)
   * @param downButton Button number for moving climber down (uses stall detection)
   * @param resetButton Button number to reset AtTop and AtBottom flags
   * @param leftEnableButton Button number that must be held to enable left climber
   * @param rightEnableButton Button number that must be held to enable right climber
   */
  public DigitalClimberCommand(
      ClimberSubsystem climberSubsystem,
      GenericHID buttonBoard,
      int upButton,
      int downButton,
      int resetButton,
      int leftEnableButton,
      int rightEnableButton) {
    this.climberSubsystem = climberSubsystem;
    this.buttonBoard = buttonBoard;
    this.upButton = upButton;
    this.downButton = downButton;
    this.resetButton = resetButton;
    this.leftEnableButton = leftEnableButton;
    this.rightEnableButton = rightEnableButton;
    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Read button states
    boolean up = buttonBoard.getRawButton(upButton);
    boolean down = buttonBoard.getRawButton(downButton);
    boolean reset = buttonBoard.getRawButton(resetButton);
    boolean leftEnabled = buttonBoard.getRawButton(leftEnableButton);
    boolean rightEnabled = buttonBoard.getRawButton(rightEnableButton);

    // Reset button pressed - clear the limit flags
    if (reset) {
      climberSubsystem.resetLimitFlags();
      return;
    }

    // No enable buttons pressed, stop the climber
    if (!leftEnabled && !rightEnabled) {
      climberSubsystem.stop();
      return;
    }

    // Determine direction and apply appropriate control
    if (up && !down) {
      // Move up using stall detection (max to top)
      climberSubsystem.maxClimberToTop(leftEnabled, rightEnabled);
    } else if (down && !up) {
      // Move down using stall detection (zero to bottom)
      climberSubsystem.zeroClimberToBottom(leftEnabled, rightEnabled);
    } else {
      // Both buttons pressed or neither pressed - stop
      climberSubsystem.stop();
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
    // Command runs continuously while trigger is active
    return false;
  }
}
