// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

public interface ClimberIO {
  public static class ClimberIOInputs {
    // Motor inputs
    public double leftMotorVoltage = 0.0;
    public double rightMotorVoltage = 0.0;
    public double leftMotorCurrent = 0.0;
    public double rightMotorCurrent = 0.0;
    public double leftMotorVelocity = 0.0;
    public double rightMotorVelocity = 0.0;

    // Servo inputs
    public double leftServoPosition = 0.0;
    public double leftServoSetpoint = 0.0;
    public double rightServoPosition = 0.0;
    public double rightServoSetpoint = 0.0;
  }

  /** Updates the set of loggable inputs */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /** Set the left motor speed (-1.0 to 1.0) */
  public default void setLeftMotor(double speed) {}

  /** Set the right motor speed (-1.0 to 1.0) */
  public default void setRightMotor(double speed) {}

  /** Set both servo positions simultaneously */
  public default void setServoPositions(double position) {}

  /** Set the left servo position (0.0 to 1.0) */
  public default void setLeftServoPosition(double position) {}

  /** Set the right servo position (0.0 to 1.0) */
  public default void setRightServoPosition(double position) {}

  /** Get the current left servo position */
  public default double getLeftServoPosition() {
    return 0.0;
  }

  /** Get the current right servo position */
  public default double getRightServoPosition() {
    return 0.0;
  }
}
