// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ClimberConstants;
import frc.robot.subsystems.Climber.ClimberSubsystem;

public class DigitalClimberCommand extends Command {
  /**
   * Creates a new DigitalClimberCommand that controls climber motors via digital buttons. Each
   * climber side has independent up/down buttons with stall detection, independent reset buttons to
   * clear AtTop/AtBottom flags, and independent servo toggle buttons.
   *
   * <p>Button layout: 1=reset left, 2=reset right, 3=left up, 4=right up, 5=left down, 6=right
   * down, 7=left servo (held=MAX, released=MIN), 8=right servo (held=MAX, released=MIN)
   */
  private final ClimberSubsystem climberSubsystem;

  private final GenericHID buttonBoard;
  private final int leftResetButton;
  private final int rightResetButton;
  private final int leftUpButton;
  private final int rightUpButton;
  private final int leftDownButton;
  private final int rightDownButton;
  private final int leftServoButton;
  private final int rightServoButton;

  /**
   * Creates a new DigitalClimberCommand.
   *
   * @param climberSubsystem The climber subsystem to control
   * @param buttonBoard The button board controller
   * @param leftResetButton Button number to reset left climber AtTop/AtBottom flags
   * @param rightResetButton Button number to reset right climber AtTop/AtBottom flags
   * @param leftUpButton Button number for moving left climber up (uses stall detection)
   * @param rightUpButton Button number for moving right climber up (uses stall detection)
   * @param leftDownButton Button number for moving left climber down (uses stall detection)
   * @param rightDownButton Button number for moving right climber down (uses stall detection)
   * @param leftServoButton Button number to control left servo (held=MAX, released=MIN)
   * @param rightServoButton Button number to control right servo (held=MAX, released=MIN)
   */
  public DigitalClimberCommand(
      ClimberSubsystem climberSubsystem,
      GenericHID buttonBoard,
      int leftResetButton,
      int rightResetButton,
      int leftUpButton,
      int rightUpButton,
      int leftDownButton,
      int rightDownButton,
      int leftServoButton,
      int rightServoButton) {
    this.climberSubsystem = climberSubsystem;
    this.buttonBoard = buttonBoard;
    this.leftResetButton = leftResetButton;
    this.rightResetButton = rightResetButton;
    this.leftUpButton = leftUpButton;
    this.rightUpButton = rightUpButton;
    this.leftDownButton = leftDownButton;
    this.rightDownButton = rightDownButton;
    this.leftServoButton = leftServoButton;
    this.rightServoButton = rightServoButton;
    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Read button states
    boolean leftReset = buttonBoard.getRawButton(leftResetButton);
    boolean rightReset = buttonBoard.getRawButton(rightResetButton);
    boolean leftUp = buttonBoard.getRawButton(leftUpButton);
    boolean rightUp = buttonBoard.getRawButton(rightUpButton);
    boolean leftDown = buttonBoard.getRawButton(leftDownButton);
    boolean rightDown = buttonBoard.getRawButton(rightDownButton);
    boolean leftServo = buttonBoard.getRawButton(leftServoButton);
    boolean rightServo = buttonBoard.getRawButton(rightServoButton);

    // --- Reset buttons ---
    if (leftReset) {
      climberSubsystem.resetLeftLimitFlags();
    }
    if (rightReset) {
      climberSubsystem.resetRightLimitFlags();
    }

    // --- Left climber motor control ---
    if (leftUp && !leftDown) {
      climberSubsystem.maxClimberToTop(true, false);
    } else if (leftDown && !leftUp) {
      climberSubsystem.zeroClimberToBottom(true, false);
    } else {
      climberSubsystem.stopLeft();
    }

    // --- Right climber motor control ---
    if (rightUp && !rightDown) {
      climberSubsystem.maxClimberToTop(false, true);
    } else if (rightDown && !rightUp) {
      climberSubsystem.zeroClimberToBottom(false, true);
    } else {
      climberSubsystem.stopRight();
    }

    // --- Servo control (held = MAX, released = MIN) ---
    climberSubsystem.setLeftServoPosition(
        leftServo ? ClimberConstants.SERVO_MAX : ClimberConstants.SERVO_MIN);
    climberSubsystem.setRightServoPosition(
        rightServo ? ClimberConstants.SERVO_MAX : ClimberConstants.SERVO_MIN);
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
