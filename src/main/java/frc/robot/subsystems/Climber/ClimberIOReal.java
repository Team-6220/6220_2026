// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.ClimberConstants;

public class ClimberIOReal implements ClimberIO {
  private final SparkMax leftMotor;
  // TODO: Add right motor when installed on full robot
  // private final SparkMax rightMotor;
  private final RelativeEncoder leftEncoder;
  // TODO: Add right encoder when installed on full robot
  // private final RelativeEncoder rightEncoder;
  private final Servo leftServo;
  private final Servo rightServo;

  public ClimberIOReal() {
    // Initialize left motor
    leftMotor = new SparkMax(ClimberConstants.climberDriverLeftID, MotorType.kBrushless);

    // TODO: Add right motor when installed on full robot
    // rightMotor = new SparkMax(ClimberConstants.climberDriverRightID, MotorType.kBrushless);

    // Configure left motor using SparkMaxConfig
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(ClimberConstants.leftMotorInverted);
    config.idleMode(IdleMode.kBrake);
    leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // TODO: Add right motor configuration when installed on full robot
    // SparkMaxConfig rightConfig = new SparkMaxConfig();
    // rightConfig.inverted(ClimberConstants.rightMotorInverted);
    // rightConfig.idleMode(IdleMode.kBrake);
    // rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters,
    // PersistMode.kPersistParameters);

    // Get encoders
    leftEncoder = leftMotor.getEncoder();

    // TODO: Add right encoders when installed on full robot
    // rightEncoder = rightMotor.getEncoder();

    // Initialize servos on PWM ports
    leftServo = new Servo(ClimberConstants.leftServoPWMPort);
    rightServo = new Servo(ClimberConstants.rightServoPWMPort);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    // Update left motor telemetry
    inputs.leftMotorVoltage = leftMotor.getBusVoltage() * leftMotor.getAppliedOutput();
    inputs.leftMotorCurrent = leftMotor.getOutputCurrent();
    inputs.leftMotorVelocity = leftEncoder.getVelocity();

    // TODO: Add right motor telemetry when installed on full robot
    // inputs.rightMotorVoltage = rightMotor.getBusVoltage() * rightMotor.getAppliedOutput();
    // inputs.rightMotorCurrent = rightMotor.getOutputCurrent();
    // inputs.rightMotorVelocity = rightEncoder.getVelocity();

    // Update servo telemetry
    inputs.leftServoPosition = leftServo.get();
    inputs.rightServoPosition = rightServo.get();
  }

  @Override
  public void setLeftMotor(double speed) {
    // nah // Clamp speed to [-0.5, 0.5] for safety
    // speed = Math.max(-0.5, Math.min(0.5, speed));
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
  public void setServoPositions(double position) {
    leftServo.set(position);
    rightServo.set(position);
  }

  @Override
  public double getLeftServoPosition() {
    return leftServo.get();
  }

  @Override
  public double getRightServoPosition() {
    return rightServo.get();
  }
}
