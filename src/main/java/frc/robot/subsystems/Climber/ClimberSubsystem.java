// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  public enum ClimberControlMode {
    LEFT, // Control left climber only
    RIGHT, // Control right climber only (not yet implemented)
    BOTH // Control both climbers together
  }

  public boolean climberAtBottom = false;

  private final ClimberIO io;
  private final ClimberIO.ClimberIOInputs inputs = new ClimberIO.ClimberIOInputs();
  private ClimberControlMode controlMode = ClimberControlMode.LEFT; // Default to left climber

  public ClimberSubsystem(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // Log current telemetry for debugging
    SmartDashboard.putNumber("Climber/CurrentHeight", inputs.leftClimberHeight);
    SmartDashboard.putNumber("Climber/EncoderPosition", inputs.leftEncoderPosition);
    SmartDashboard.putNumber("Climber/MotorCurrent", inputs.leftMotorCurrent);
    SmartDashboard.putNumber("Climber/MotorVoltage", inputs.leftMotorVoltage);
    SmartDashboard.putNumber("Climber/MotorVelocity", inputs.leftMotorVelocity);
    SmartDashboard.putNumber("Climber/ServoPosition", inputs.servoPosition);
    SmartDashboard.putString("Climber/ControlMode", controlMode.toString());
  }

  /**
   * Drive the climber motor at a specified speed (manual control)
   *
   * @param speed Motor speed from -0.75 to 0.75
   */
  public void setMotorSpeed(double speed) {
    // Clamp to safe range
    speed = Math.max(-0.75, Math.min(0.75, speed));
    io.setLeftMotor(speed);
  }

  /**
   * Automatically zero the climber by driving down until stall current is detected. The motor will
   * be driven downward until current exceeds the stall threshold, indicating the climber has hit
   * the bottom.
   */
  public void zeroClimberToBottom() {
    // Drive downward (negative speed)
    double downwardSpeed = -0.75;

    // Check if motor is stalled (high current draw)
    if (inputs.leftMotorCurrent > ClimberConstants.STALL_CURRENT_THRESHOLD
        & inputs.leftMotorVelocity < ClimberConstants.VELOCITY_THRESHOLD) {
      // Motor is stalled, we've hit bottom - stop
      io.setLeftMotor(0.0);
    } else {
      // Continue driving downward
      io.setLeftMotor(downwardSpeed);
    }
  }

  /** Check if climber is at zero position (stalled) */
  public boolean isAtZero() {
    return inputs.leftMotorCurrent > ClimberConstants.STALL_CURRENT_THRESHOLD
        & inputs.leftMotorVelocity < ClimberConstants.VELOCITY_THRESHOLD;
  }

  /**
   * Set the climber servo position
   *
   * @param position Servo position (0.0 to 1.0)
   */
  public void setServoPosition(double position) {
    io.setServoPosition(position);
  }

  /**
   * Get the current servo position
   *
   * @return Current position from 0.0 to 1.0
   */
  public double getServoPosition() {
    return inputs.servoPosition;
  }

  /** Stop the climber motor */
  public void stop() {
    io.setLeftMotor(0.0);
  }

  /** Toggle the climber control mode to the next mode (LEFT -> RIGHT -> BOTH -> LEFT) */
  public void toggleControlMode() {
    switch (controlMode) {
      case LEFT:
        controlMode = ClimberControlMode.RIGHT;
        break;
      case RIGHT:
        controlMode = ClimberControlMode.BOTH;
        break;
      case BOTH:
        controlMode = ClimberControlMode.LEFT;
        break;
    }
    // Log the new mode to SmartDashboard
    SmartDashboard.putString("Climber/ControlMode", controlMode.toString());
  }

  /**
   * Get the current climber control mode
   *
   * @return Current control mode (LEFT, RIGHT, or BOTH)
   */
  public ClimberControlMode getControlMode() {
    return controlMode;
  }

  /**
   * Set the climber control mode
   *
   * @param mode The desired control mode
   */
  public void setControlMode(ClimberControlMode mode) {
    controlMode = mode;
    SmartDashboard.putString("Climber/ControlMode", controlMode.toString());
  }
}
