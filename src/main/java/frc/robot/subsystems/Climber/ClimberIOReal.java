// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.ClimberConstants;

public class ClimberIOReal implements ClimberIO {
    private final CANSparkMax leftMotor;
    // TODO: Add right motor when installed on full robot
    // private final CANSparkMax rightMotor;
    private final RelativeEncoder leftEncoder;
    // TODO: Add right encoder when installed on full robot
    // private final RelativeEncoder rightEncoder;
    private final AbsoluteEncoder leftAbsoluteEncoder;
    // TODO: Add right absolute encoder when installed on full robot
    // private final AbsoluteEncoder rightAbsoluteEncoder;
    private final Servo servo;
    
    public ClimberIOReal() {
        // Initialize left motor
        leftMotor = new CANSparkMax(ClimberConstants.climberDriverLeftID, MotorType.kBrushless);
        
        // TODO: Add right motor when installed on full robot
        // rightMotor = new CANSparkMax(ClimberConstants.climberDriverRightID, MotorType.kBrushless);
        
        // Configure left motor
        leftMotor.restoreFactoryDefaults();
        leftMotor.setInverted(ClimberConstants.motorAInverted);
        leftMotor.setIdleMode(IdleMode.kBrake);
        
        // TODO: Add right motor configuration when installed on full robot
        // rightMotor.restoreFactoryDefaults();
        // rightMotor.setInverted(ClimberConstants.motorBInverted);
        // rightMotor.setIdleMode(IdleMode.kBrake);
        
        // Get encoders
        leftEncoder = leftMotor.getEncoder();
        leftAbsoluteEncoder = leftMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        
        // TODO: Add right encoders when installed on full robot
        // rightEncoder = rightMotor.getEncoder();
        // rightAbsoluteEncoder = rightMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        
        // Burn flash to save config
        leftMotor.burnFlash();
        // TODO: Add right motor flash when installed on full robot
        // rightMotor.burnFlash();
        
        // Initialize servo on PWM port
        servo = new Servo(ClimberConstants.servoPWMPort);
    }
    
    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        // Update left motor telemetry
        inputs.leftMotorVoltage = leftMotor.getBusVoltage() * leftMotor.getAppliedOutput();
        inputs.leftMotorCurrent = leftMotor.getOutputCurrent();
        inputs.leftMotorVelocity = leftEncoder.getVelocity();
        inputs.leftMotorPosition = leftEncoder.getPosition();
        
        // Update left encoder telemetry
        inputs.leftEncoderPosition = leftAbsoluteEncoder.getPosition();
        inputs.leftEncoderVelocity = leftAbsoluteEncoder.getVelocity();
        inputs.leftClimberHeight = inputs.leftEncoderPosition * ClimberConstants.climberHeightCoefficient;
        
        // TODO: Add right motor telemetry when installed on full robot
        // inputs.rightMotorVoltage = rightMotor.getBusVoltage() * rightMotor.getAppliedOutput();
        // inputs.rightMotorCurrent = rightMotor.getOutputCurrent();
        // inputs.rightMotorVelocity = rightEncoder.getVelocity();
        // inputs.rightMotorPosition = rightEncoder.getPosition();
        
        // TODO: Add right encoder telemetry when installed on full robot
        // inputs.rightEncoderPosition = rightAbsoluteEncoder.getPosition();
        // inputs.rightEncoderVelocity = rightAbsoluteEncoder.getVelocity();
        // inputs.rightClimberHeight = inputs.rightEncoderPosition * ClimberConstants.climberHeightCoefficient;
        
        // Update servo telemetry
        inputs.servoPosition = servo.get();
    }
    
    @Override
    public void setLeftMotor(double speed) {
        // Clamp speed to [-0.5, 0.5] for safety
        speed = Math.max(-0.5, Math.min(0.5, speed));
        leftMotor.set(speed);
    }
    
    // TODO: Add right motor control when installed on full robot
    // @Override
    // public void setRightMotor(double speed) {
    //     // Clamp speed to [-0.5, 0.5] for safety
    //     speed = Math.max(-0.5, Math.min(0.5, speed));
    //     rightMotor.set(speed);
    // }
    
    @Override
    public void setServoPosition(double position) {
        servo.set(position);
    }
    
    @Override
    public double getServoPosition() {
        return servo.get();
    }
}