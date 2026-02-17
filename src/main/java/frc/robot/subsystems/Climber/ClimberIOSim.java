// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

public class ClimberIOSim implements ClimberIO {
  private double leftMotorSpeed = 0.0;
  // TODO: Add right motor when installed on full robot
  // private double rightMotorSpeed = 0.0;
  private double leftServoPosition = 0.5; // Start at center
  private double rightServoPosition = 0.5; // Start at center

  public ClimberIOSim() {}

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    // Update left motor inputs
    inputs.leftMotorVoltage = leftMotorSpeed * 12.0;
    inputs.leftMotorCurrent = Math.abs(leftMotorSpeed) * 20.0; // Simulated current
    inputs.leftMotorVelocity = leftMotorSpeed * 5000.0; // Simulated RPM

    // TODO: Add right motor inputs when installed on full robot
    // inputs.rightMotorVoltage = rightMotorSpeed * 12.0;
    // inputs.rightMotorCurrent = Math.abs(rightMotorSpeed) * 20.0;
    // inputs.rightMotorVelocity = rightMotorSpeed * 5000.0;

    // Update servo inputs
    inputs.leftServoPosition = leftServoPosition;
    inputs.leftServoSetpoint = leftServoPosition;
    inputs.rightServoPosition = rightServoPosition;
    inputs.rightServoSetpoint = rightServoPosition;
  }

  @Override
  public void setLeftMotor(double speed) {
    this.leftMotorSpeed = Math.max(-0.75, Math.min(0.75, speed));
  }

  // TODO: Add right motor control when installed on full robot
  // @Override
  // public void setRightMotor(double speed) {
  //     this.rightMotorSpeed = Math.max(-0.75, Math.min(0.75, speed));
  // }

  @Override
  public void setServoPositions(double position) {
    this.leftServoPosition = Math.max(0.0, Math.min(1.0, position));
    this.rightServoPosition = Math.max(0.0, Math.min(1.0, position));
  }

  @Override
  public double getLeftServoPosition() {
    return leftServoPosition;
  }

  @Override
  public double getRightServoPosition() {
    return rightServoPosition;
  }
}
