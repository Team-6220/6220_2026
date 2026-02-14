// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

public interface ClimberIO {
  public static class ClimberIOInputs {
    // Motor inputs
    public double leftMotorVoltage = 0.0;
    // TODO: Add right motor when installed on full robot
    // public double rightMotorVoltage = 0.0;
    public double leftMotorCurrent = 0.0;
    // TODO: Add right motor when installed on full robot
    // public double rightMotorCurrent = 0.0;
    public double leftMotorVelocity = 0.0;
    // TODO: Add right motor when installed on full robot
    // public double rightMotorVelocity = 0.0;
    public double leftMotorPosition = 0.0;
    // TODO: Add right motor when installed on full robot
    // public double rightMotorPosition = 0.0;
    public double leftMotorSpeed = 0.0;

    // Absolute encoder inputs
    public double leftEncoderPosition = 0.0; // Raw encoder position in rotations
    public double leftEncoderVelocity = 0.0; // Encoder velocity in rotations per second
    public double leftClimberHeight = 0.0; // Calculated height in inches/meters
    // TODO: Add right encoder when installed on full robot
    // public double rightEncoderPosition = 0.0;
    // public double rightEncoderVelocity = 0.0;
    // public double rightClimberHeight = 0.0;

    // Servo inputs
    public double servoPosition = 0.0;
    public double servoSetpoint = 0.0;
  }

  /** Updates the set of loggable inputs */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /** Set the left motor speed (-1.0 to 1.0) */
  public default void setLeftMotor(double speed) {}

  // TODO: Add right motor when installed on full robot
  // /** Set the right motor speed (-1.0 to 1.0) */
  // public default void setRightMotor(double speed) {}

  /** Set the servo position */
  public default void setServoPosition(double position) {}

  /** Get the current servo position */
  public default double getServoPosition() {
    return 0.0;
  }
}
