// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.revrobotics.AbsoluteEncoder;
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
  private final AbsoluteEncoder leftAbsoluteEncoder;
  // TODO: Add right absolute encoder when installed on full robot
  // private final AbsoluteEncoder rightAbsoluteEncoder;
  private final Servo servo;

  // Unwrapping encoder position (handles 0-1 wrapping)
  private double previousAbsolutePosition = 0.0;
  private int fullRotationCount;

  public ClimberIOReal() {
    // Initialize left motor
    leftMotor = new SparkMax(ClimberConstants.climberDriverLeftID, MotorType.kBrushless);

    // TODO: Add right motor when installed on full robot
    // rightMotor = new SparkMax(ClimberConstants.climberDriverRightID, MotorType.kBrushless);

    // Configure left motor using SparkMaxConfig
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(ClimberConstants.motorAInverted);
    config.idleMode(IdleMode.kBrake);
    leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // TODO: Add right motor configuration when installed on full robot
    // SparkMaxConfig rightConfig = new SparkMaxConfig();
    // rightConfig.inverted(ClimberConstants.motorBInverted);
    // rightConfig.idleMode(IdleMode.kBrake);
    // rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters,
    // PersistMode.kPersistParameters);

    // Get encoders
    leftEncoder = leftMotor.getEncoder();
    leftAbsoluteEncoder = leftMotor.getAbsoluteEncoder();

    // TODO: Add right encoders when installed on full robot
    // rightEncoder = rightMotor.getEncoder();
    // rightAbsoluteEncoder = rightMotor.getAbsoluteEncoder();

    // Initialize servo on PWM port
    servo = new Servo(ClimberConstants.servoPWMPort);

    // Initialize with offset from constants (convert inches to rotations)
    fullRotationCount =
        (int)
            (ClimberConstants.climberEncoderOffsetInches
                / ClimberConstants.climberHeightCoefficient);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    // Update left motor telemetry
    inputs.leftMotorVoltage = leftMotor.getBusVoltage() * leftMotor.getAppliedOutput();
    inputs.leftMotorCurrent = leftMotor.getOutputCurrent();
    inputs.leftMotorVelocity = leftEncoder.getVelocity();
    inputs.leftMotorPosition = leftEncoder.getPosition();

    // Update left encoder telemetry with unwrapping to handle 0-1 rotation wrapping
    double rawAbsolutePosition = leftAbsoluteEncoder.getPosition();

    // Detect wrapping and unwrap the position
    double delta = rawAbsolutePosition - previousAbsolutePosition;
    if (delta < -0.5) {
      // Wrapped from 1 to 0
      fullRotationCount++;
    } else if (delta > 0.5) {
      // Wrapped from 0 to 1 (going backwards)
      fullRotationCount--;
    }
    previousAbsolutePosition = rawAbsolutePosition;

    // Calculate unwrapped position in rotations
    double unwrappedRotations = fullRotationCount + rawAbsolutePosition;

    // Convert rotations to height in inches
    inputs.leftEncoderPosition = unwrappedRotations; // Store raw rotations
    inputs.leftEncoderVelocity = leftAbsoluteEncoder.getVelocity();
    inputs.leftClimberHeight = unwrappedRotations * ClimberConstants.climberHeightCoefficient;

    // TODO: Add right motor telemetry when installed on full robot
    // inputs.rightMotorVoltage = rightMotor.getBusVoltage() * rightMotor.getAppliedOutput();
    // inputs.rightMotorCurrent = rightMotor.getOutputCurrent();
    // inputs.rightMotorVelocity = rightEncoder.getVelocity();
    // inputs.rightMotorPosition = rightEncoder.getPosition();

    // TODO: Add right encoder telemetry when installed on full robot
    // inputs.rightEncoderPosition = rightAbsoluteEncoder.getPosition();
    // inputs.rightEncoderVelocity = rightAbsoluteEncoder.getVelocity();
    // inputs.rightClimberHeight = inputs.rightEncoderPosition *
    // ClimberConstants.climberHeightCoefficient;

    // Update servo telemetry
    inputs.servoPosition = servo.get();
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
  public void setServoPosition(double position) {
    servo.set(position);
  }

  @Override
  public double getServoPosition() {
    return servo.get();
  }
}
