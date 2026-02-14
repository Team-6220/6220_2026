// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import frc.robot.Constants.ClimberConstants;

public class ClimberIOSim implements ClimberIO {
    private double leftMotorSpeed = 0.0;
    // TODO: Add right motor when installed on full robot
    // private double rightMotorSpeed = 0.0;
    private double leftMotorPosition = 0.0;
    // TODO: Add right motor when installed on full robot
    // private double rightMotorPosition = 0.0;
    private double leftEncoderPosition = 0.0;
    // TODO: Add right encoder when installed on full robot
    // private double rightEncoderPosition = 0.0;
    private double servoPosition = 0.5; // Start at center
    
    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        // Simulate motor positions based on speed
        leftMotorPosition += leftMotorSpeed * 0.02; // Assuming 20ms loop time
        leftEncoderPosition += leftMotorSpeed * 0.02; // Simulate absolute encoder
        // TODO: Add right motor simulation when installed on full robot
        // rightMotorPosition += rightMotorSpeed * 0.02;
        // rightEncoderPosition += rightMotorSpeed * 0.02;
        
        // Update left motor inputs
        inputs.leftMotorVoltage = leftMotorSpeed * 12.0;
        inputs.leftMotorCurrent = Math.abs(leftMotorSpeed) * 20.0; // Simulated current
        inputs.leftMotorVelocity = leftMotorSpeed * 5000.0; // Simulated RPM
        inputs.leftMotorPosition = leftMotorPosition;
        
        // Update left encoder inputs
        inputs.leftEncoderPosition = leftEncoderPosition;
        inputs.leftEncoderVelocity = leftMotorSpeed * 5000.0 / 60.0; // Convert RPM to rotations/sec
        inputs.leftClimberHeight = leftEncoderPosition * ClimberConstants.climberHeightCoefficient;
        
        // TODO: Add right motor inputs when installed on full robot
        // inputs.rightMotorVoltage = rightMotorSpeed * 12.0;
        // inputs.rightMotorCurrent = Math.abs(rightMotorSpeed) * 20.0;
        // inputs.rightMotorVelocity = rightMotorSpeed * 5000.0;
        // inputs.rightMotorPosition = rightMotorPosition;
        
        // TODO: Add right encoder inputs when installed on full robot
        // inputs.rightEncoderPosition = rightEncoderPosition;
        // inputs.rightEncoderVelocity = rightMotorSpeed * 5000.0 / 60.0;
        // inputs.rightClimberHeight = rightEncoderPosition * ClimberConstants.climberHeightCoefficient;
        
        // Update servo inputs
        inputs.servoPosition = servoPosition;
        inputs.servoSetpoint = servoPosition;
    }
    
    @Override
    public void setLeftMotor(double speed) {
        this.leftMotorSpeed = Math.max(-0.5, Math.min(0.5, speed));
    }
    
    // TODO: Add right motor control when installed on full robot
    // @Override
    // public void setRightMotor(double speed) {
    //     this.rightMotorSpeed = Math.max(-0.5, Math.min(0.5, speed));
    // }
    
    @Override
    public void setServoPosition(double position) {
        this.servoPosition = Math.max(0.0, Math.min(1.0, position));
    }
    
    @Override
    public double getServoPosition() {
        return servoPosition;
    }
}