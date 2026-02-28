// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TunableNumber;

public class ShooterSubsystem extends SubsystemBase {

  // Motor CAN IDs
  // Motors 41 and 1 face the opposite direction (inverted)
  // Motors 9 and 31 spin the same way (not inverted)
  private static final int MOTOR_41_ID = 41;
  private static final int MOTOR_1_ID = 1;
  private static final int MOTOR_9_ID = 9;
  private static final int MOTOR_31_ID = 31;

  // Motors
  private final TalonFX m_motor41; // inverted
  private final TalonFX m_motor1; // inverted
  private final TalonFX m_motor9; // not inverted
  private final TalonFX m_motor31; // not inverted

  // Velocity control request
  private final VelocityVoltage m_velocityRequest;

  // Tunable PID and feedforward values
  private final TunableNumber m_kp = new TunableNumber("Shooter/kP", 0.1);
  private final TunableNumber m_ki = new TunableNumber("Shooter/kI", 0.0);
  private final TunableNumber m_kd = new TunableNumber("Shooter/kD", 0.0);
  private final TunableNumber m_kv = new TunableNumber("Shooter/kV", 0.12);
  private final TunableNumber m_ks = new TunableNumber("Shooter/kS", 0.0);
  private final TunableNumber m_ka = new TunableNumber("Shooter/kA", 0.0);

  // Tunable target velocity
  private final TunableNumber m_targetVelocityRPS =
      new TunableNumber("Shooter/TargetVelocityRPS", 60.0);

  // Tolerance for determining if shooter is at speed (in RPS)
  private static final double VELOCITY_TOLERANCE_RPS = 2.0;

  // Current limit in amps
  private static final double CURRENT_LIMIT = 40.0;

  public ShooterSubsystem() {
    m_motor41 = new TalonFX(MOTOR_41_ID);
    m_motor1 = new TalonFX(MOTOR_1_ID);
    m_motor9 = new TalonFX(MOTOR_9_ID);
    m_motor31 = new TalonFX(MOTOR_31_ID);

    m_velocityRequest = new VelocityVoltage(0).withSlot(0);

    configureMotors();
  }

  private void configureMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Current limits
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
    config.CurrentLimits = currentLimits;

    // PID + feedforward (Slot 0)
    Slot0Configs pidConfigs = new Slot0Configs();
    pidConfigs.kP = m_kp.get();
    pidConfigs.kI = m_ki.get();
    pidConfigs.kD = m_kd.get();
    pidConfigs.kV = m_kv.get();
    pidConfigs.kS = m_ks.get();
    pidConfigs.kA = m_ka.get();
    config.Slot0 = pidConfigs;

    // Motor output config
    MotorOutputConfigs outputConfig = new MotorOutputConfigs();
    outputConfig.NeutralMode = NeutralModeValue.Coast;

    // Motors 41 and 1 face opposite direction -> Clockwise Positive (inverted)
    outputConfig.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput = outputConfig;
    m_motor41.getConfigurator().apply(config);
    m_motor1.getConfigurator().apply(config);

    // Motors 9 and 31 spin the same way -> CounterClockwise Positive (default)
    outputConfig.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput = outputConfig;
    m_motor9.getConfigurator().apply(config);
    m_motor31.getConfigurator().apply(config);
  }

  /** Runs all shooter motors at the tunable target velocity. */
  public void runAtTargetVelocity() {
    double rps = m_targetVelocityRPS.get();
    setVelocityRPS(rps);
  }

  /**
   * Sets all shooter motors to the given velocity in RPS.
   *
   * @param rps Target velocity in rotations per second
   */
  public void setVelocityRPS(double rps) {
    m_motor41.setControl(m_velocityRequest.withVelocity(rps));
    m_motor1.setControl(m_velocityRequest.withVelocity(rps));
    m_motor9.setControl(m_velocityRequest.withVelocity(rps));
    m_motor31.setControl(m_velocityRequest.withVelocity(rps));
  }

  /**
   * Sets all shooter motors to the given velocity in RPM.
   *
   * @param rpm Target velocity in rotations per minute
   */
  public void setVelocityRPM(double rpm) {
    setVelocityRPS(rpm / 60.0);
  }

  /** Stops all shooter motors. */
  public void stop() {
    m_motor41.stopMotor();
    m_motor1.stopMotor();
    m_motor9.stopMotor();
    m_motor31.stopMotor();
  }

  /**
   * Runs all shooters at a percentage of max output (for testing).
   *
   * @param percent Percent output from -1.0 to 1.0
   */
  public void setPercentOutput(double percent) {
    m_motor41.set(percent);
    m_motor1.set(percent);
    m_motor9.set(percent);
    m_motor31.set(percent);
  }

  // ========== Velocity Getters ==========

  public double getMotor41VelocityRPS() {
    return m_motor41.getVelocity().getValueAsDouble();
  }

  public double getMotor1VelocityRPS() {
    return m_motor1.getVelocity().getValueAsDouble();
  }

  public double getMotor9VelocityRPS() {
    return m_motor9.getVelocity().getValueAsDouble();
  }

  public double getMotor31VelocityRPS() {
    return m_motor31.getVelocity().getValueAsDouble();
  }

  /**
   * Gets the average velocity of all four shooter motors.
   *
   * @return Average velocity in RPS
   */
  public double getAverageVelocityRPS() {
    return (getMotor41VelocityRPS()
            + getMotor1VelocityRPS()
            + getMotor9VelocityRPS()
            + getMotor31VelocityRPS())
        / 4.0;
  }

  /**
   * Checks if all shooter motors are within tolerance of the target velocity.
   *
   * @return True if all motors are at speed
   */
  public boolean isAtSpeed() {
    double target = m_targetVelocityRPS.get();
    if (target == 0.0) {
      return false;
    }

    return Math.abs(getMotor41VelocityRPS() - target) < VELOCITY_TOLERANCE_RPS
        && Math.abs(getMotor1VelocityRPS() - target) < VELOCITY_TOLERANCE_RPS
        && Math.abs(getMotor9VelocityRPS() - target) < VELOCITY_TOLERANCE_RPS
        && Math.abs(getMotor31VelocityRPS() - target) < VELOCITY_TOLERANCE_RPS;
  }

  /**
   * Gets the current target velocity.
   *
   * @return Target velocity in RPS
   */
  public double getTargetVelocityRPS() {
    return m_targetVelocityRPS.get();
  }

  /** Updates PID values from tunable numbers (only Slot0, avoids full reconfigure). */
  private void updatePIDValues() {
    Slot0Configs pidConfigs = new Slot0Configs();
    pidConfigs.kP = m_kp.get();
    pidConfigs.kI = m_ki.get();
    pidConfigs.kD = m_kd.get();
    pidConfigs.kV = m_kv.get();
    pidConfigs.kS = m_ks.get();
    pidConfigs.kA = m_ka.get();

    m_motor41.getConfigurator().apply(pidConfigs);
    m_motor1.getConfigurator().apply(pidConfigs);
    m_motor9.getConfigurator().apply(pidConfigs);
    m_motor31.getConfigurator().apply(pidConfigs);
  }

  /**
   * Sets the neutral mode for all motors.
   *
   * @param mode Brake or Coast
   */
  public void setNeutralMode(NeutralModeValue mode) {
    MotorOutputConfigs output = new MotorOutputConfigs();
    output.NeutralMode = mode;

    m_motor41.getConfigurator().apply(output);
    m_motor1.getConfigurator().apply(output);
    m_motor9.getConfigurator().apply(output);
    m_motor31.getConfigurator().apply(output);
  }

  @Override
  public void periodic() {
    // Update PID if tunable numbers changed
    if (m_kp.hasChanged()
        || m_ki.hasChanged()
        || m_kd.hasChanged()
        || m_kv.hasChanged()
        || m_ks.hasChanged()
        || m_ka.hasChanged()) {
      updatePIDValues();
    }

    // Telemetry
    SmartDashboard.putNumber("Shooter/Motor41VelocityRPS", getMotor41VelocityRPS());
    SmartDashboard.putNumber("Shooter/Motor1VelocityRPS", getMotor1VelocityRPS());
    SmartDashboard.putNumber("Shooter/Motor9VelocityRPS", getMotor9VelocityRPS());
    SmartDashboard.putNumber("Shooter/Motor31VelocityRPS", getMotor31VelocityRPS());
    SmartDashboard.putNumber("Shooter/AverageVelocityRPS", getAverageVelocityRPS());
    SmartDashboard.putNumber("Shooter/TargetVelocityRPS", m_targetVelocityRPS.get());
    SmartDashboard.putBoolean("Shooter/AtSpeed", isAtSpeed());
  }
}
