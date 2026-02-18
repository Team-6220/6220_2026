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
  
  // Motor CAN IDs - adjust these to match your robot
  // Shooter 1 (Left)
  private static final int SHOOTER1_TOP_ID = 10;
  private static final int SHOOTER1_BOTTOM_ID = 11;
  
  // Shooter 2 (Center)
  private static final int SHOOTER2_TOP_ID = 12;
  private static final int SHOOTER2_BOTTOM_ID = 13;
  
  // Shooter 3 (Right)
  private static final int SHOOTER3_TOP_ID = 14;
  private static final int SHOOTER3_BOTTOM_ID = 15;

  // Motors for all 3 shooters
  private final TalonFX m_shooter1Top;
  private final TalonFX m_shooter1Bottom;
  private final TalonFX m_shooter2Top;
  private final TalonFX m_shooter2Bottom;
  private final TalonFX m_shooter3Top;
  private final TalonFX m_shooter3Bottom;

  // Velocity control requests
  private final VelocityVoltage m_velocityRequest;

  // Tunable PID values (shared across all shooters)
  private final TunableNumber m_kp = new TunableNumber("Shooter/kP", 0.1);
  private final TunableNumber m_ki = new TunableNumber("Shooter/kI", 0.0);
  private final TunableNumber m_kd = new TunableNumber("Shooter/kD", 0.0);
  private final TunableNumber m_kv = new TunableNumber("Shooter/kV", 0.12);
  private final TunableNumber m_ks = new TunableNumber("Shooter/kS", 0.0);

  // Target velocity in rotations per second (RPS)
  private double m_targetVelocityRPS = 0.0;

  // Tolerance for determining if shooter is at speed (in RPS)
  private static final double VELOCITY_TOLERANCE_RPS = 2.0;

  // Current limit in amps
  private static final double CURRENT_LIMIT = 40.0;

  // Enum for selecting which shooters to use
  public enum ShooterSelection {
    ALL,           // All 3 shooters
    SHOOTER_1,     // Left shooter only
    SHOOTER_2,     // Center shooter only
    SHOOTER_3,     // Right shooter only
    SHOOTERS_1_2,  // Left + Center
    SHOOTERS_2_3,  // Center + Right
    SHOOTERS_1_3   // Left + Right
  }

  private ShooterSelection m_activeShooters = ShooterSelection.ALL;

  public ShooterSubsystem() {
    // Initialize all motors
    m_shooter1Top = new TalonFX(SHOOTER1_TOP_ID);
    m_shooter1Bottom = new TalonFX(SHOOTER1_BOTTOM_ID);
    m_shooter2Top = new TalonFX(SHOOTER2_TOP_ID);
    m_shooter2Bottom = new TalonFX(SHOOTER2_BOTTOM_ID);
    m_shooter3Top = new TalonFX(SHOOTER3_TOP_ID);
    m_shooter3Bottom = new TalonFX(SHOOTER3_BOTTOM_ID);

    // Initialize velocity request
    m_velocityRequest = new VelocityVoltage(0).withSlot(0);

    configureMotors();
  }

  private void configureMotors() {
    // Create configuration objects
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Configure current limits
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
    config.CurrentLimits = currentLimits;

    // Configure motor output
    MotorOutputConfigs outputConfig = new MotorOutputConfigs();
    outputConfig.NeutralMode = NeutralModeValue.Coast;

    // Configure PID gains (Slot 0)
    Slot0Configs pidConfigs = new Slot0Configs();
    pidConfigs.kP = m_kp.get();
    pidConfigs.kI = m_ki.get();
    pidConfigs.kD = m_kd.get();
    pidConfigs.kV = m_kv.get();
    pidConfigs.kS = m_ks.get();
    config.Slot0 = pidConfigs;

    // Apply configurations to all motors
    // Top motors - clockwise positive
    outputConfig.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput = outputConfig;
    m_shooter1Top.getConfigurator().apply(config);
    m_shooter2Top.getConfigurator().apply(config);
    m_shooter3Top.getConfigurator().apply(config);

    // Bottom motors - counter-clockwise positive
    outputConfig.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput = outputConfig;
    m_shooter1Bottom.getConfigurator().apply(config);
    m_shooter2Bottom.getConfigurator().apply(config);
    m_shooter3Bottom.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    // Update PID values if they've changed
    if (m_kp.hasChanged() || m_ki.hasChanged() || m_kd.hasChanged() || 
        m_kv.hasChanged() || m_ks.hasChanged()) {
      updatePIDValues();
    }

    // Log telemetry for all shooters
    SmartDashboard.putNumber("Shooter1/TopVelocityRPS", getShooter1TopVelocityRPS());
    SmartDashboard.putNumber("Shooter1/BottomVelocityRPS", getShooter1BottomVelocityRPS());
    SmartDashboard.putNumber("Shooter2/TopVelocityRPS", getShooter2TopVelocityRPS());
    SmartDashboard.putNumber("Shooter2/BottomVelocityRPS", getShooter2BottomVelocityRPS());
    SmartDashboard.putNumber("Shooter3/TopVelocityRPS", getShooter3TopVelocityRPS());
    SmartDashboard.putNumber("Shooter3/BottomVelocityRPS", getShooter3BottomVelocityRPS());
    
    SmartDashboard.putNumber("Shooter/TargetVelocityRPS", m_targetVelocityRPS);
    SmartDashboard.putBoolean("Shooter/AllAtSpeed", isAtSpeed());
    SmartDashboard.putString("Shooter/ActiveShooters", m_activeShooters.toString());
  }

  /**
   * Sets which shooters should be active
   * @param selection Which shooter(s) to use
   */
  public void setActiveShooters(ShooterSelection selection) {
    m_activeShooters = selection;
  }

  /**
   * Gets the current active shooter selection
   * @return Current active shooters
   */
  public ShooterSelection getActiveShooters() {
    return m_activeShooters;
  }

  /**
   * Sets the target velocity for all active shooters
   * @param rpm Target velocity in RPM
   */
  public void setTargetRPM(double rpm) {
    setTargetRPS(rpm / 60.0);
  }

  /**
   * Sets the target velocity for all active shooters
   * @param rps Target velocity in rotations per second
   */
  public void setTargetRPS(double rps) {
    m_targetVelocityRPS = rps;
    
    // Apply velocity to active shooters based on selection
    switch (m_activeShooters) {
      case ALL:
        setAllShootersRPS(rps);
        break;
      case SHOOTER_1:
        setShooter1RPS(rps);
        stopShooter2();
        stopShooter3();
        break;
      case SHOOTER_2:
        stopShooter1();
        setShooter2RPS(rps);
        stopShooter3();
        break;
      case SHOOTER_3:
        stopShooter1();
        stopShooter2();
        setShooter3RPS(rps);
        break;
      case SHOOTERS_1_2:
        setShooter1RPS(rps);
        setShooter2RPS(rps);
        stopShooter3();
        break;
      case SHOOTERS_2_3:
        stopShooter1();
        setShooter2RPS(rps);
        setShooter3RPS(rps);
        break;
      case SHOOTERS_1_3:
        setShooter1RPS(rps);
        stopShooter2();
        setShooter3RPS(rps);
        break;
    }
  }

  /**
   * Stops all shooters
   */
  public void stop() {
    m_targetVelocityRPS = 0.0;
    stopAllShooters();
  }

  /**
   * Runs all shooters at a percentage of max output (for testing)
   * @param percent Percent output from -1.0 to 1.0
   */
  public void setPercentOutput(double percent) {
    m_shooter1Top.set(percent);
    m_shooter1Bottom.set(percent);
    m_shooter2Top.set(percent);
    m_shooter2Bottom.set(percent);
    m_shooter3Top.set(percent);
    m_shooter3Bottom.set(percent);
  }

  // ========== Individual Shooter Control ==========
  
  private void setShooter1RPS(double rps) {
    m_shooter1Top.setControl(m_velocityRequest.withVelocity(rps));
    m_shooter1Bottom.setControl(m_velocityRequest.withVelocity(rps));
  }

  private void setShooter2RPS(double rps) {
    m_shooter2Top.setControl(m_velocityRequest.withVelocity(rps));
    m_shooter2Bottom.setControl(m_velocityRequest.withVelocity(rps));
  }

  private void setShooter3RPS(double rps) {
    m_shooter3Top.setControl(m_velocityRequest.withVelocity(rps));
    m_shooter3Bottom.setControl(m_velocityRequest.withVelocity(rps));
  }

  private void setAllShootersRPS(double rps) {
    setShooter1RPS(rps);
    setShooter2RPS(rps);
    setShooter3RPS(rps);
  }

  private void stopShooter1() {
    m_shooter1Top.stopMotor();
    m_shooter1Bottom.stopMotor();
  }

  private void stopShooter2() {
    m_shooter2Top.stopMotor();
    m_shooter2Bottom.stopMotor();
  }

  private void stopShooter3() {
    m_shooter3Top.stopMotor();
    m_shooter3Bottom.stopMotor();
  }

  private void stopAllShooters() {
    stopShooter1();
    stopShooter2();
    stopShooter3();
  }

  // ========== Velocity Getters ==========
  
  public double getShooter1TopVelocityRPS() {
    return m_shooter1Top.getVelocity().getValueAsDouble();
  }

  public double getShooter1BottomVelocityRPS() {
    return m_shooter1Bottom.getVelocity().getValueAsDouble();
  }

  public double getShooter2TopVelocityRPS() {
    return m_shooter2Top.getVelocity().getValueAsDouble();
  }

  public double getShooter2BottomVelocityRPS() {
    return m_shooter2Bottom.getVelocity().getValueAsDouble();
  }

  public double getShooter3TopVelocityRPS() {
    return m_shooter3Top.getVelocity().getValueAsDouble();
  }

  public double getShooter3BottomVelocityRPS() {
    return m_shooter3Bottom.getVelocity().getValueAsDouble();
  }

  /**
   * Gets the average velocity of all active shooter wheels
   * @return Average velocity in RPS
   */
  public double getAverageVelocityRPS() {
    double sum = 0;
    int count = 0;

    switch (m_activeShooters) {
      case ALL:
        sum = getShooter1TopVelocityRPS() + getShooter1BottomVelocityRPS() +
              getShooter2TopVelocityRPS() + getShooter2BottomVelocityRPS() +
              getShooter3TopVelocityRPS() + getShooter3BottomVelocityRPS();
        count = 6;
        break;
      case SHOOTER_1:
        sum = getShooter1TopVelocityRPS() + getShooter1BottomVelocityRPS();
        count = 2;
        break;
      case SHOOTER_2:
        sum = getShooter2TopVelocityRPS() + getShooter2BottomVelocityRPS();
        count = 2;
        break;
      case SHOOTER_3:
        sum = getShooter3TopVelocityRPS() + getShooter3BottomVelocityRPS();
        count = 2;
        break;
      case SHOOTERS_1_2:
        sum = getShooter1TopVelocityRPS() + getShooter1BottomVelocityRPS() +
              getShooter2TopVelocityRPS() + getShooter2BottomVelocityRPS();
        count = 4;
        break;
      case SHOOTERS_2_3:
        sum = getShooter2TopVelocityRPS() + getShooter2BottomVelocityRPS() +
              getShooter3TopVelocityRPS() + getShooter3BottomVelocityRPS();
        count = 4;
        break;
      case SHOOTERS_1_3:
        sum = getShooter1TopVelocityRPS() + getShooter1BottomVelocityRPS() +
              getShooter3TopVelocityRPS() + getShooter3BottomVelocityRPS();
        count = 4;
        break;
    }

    return count > 0 ? sum / count : 0;
  }

  /**
   * Checks if all active shooters are at the target speed
   * @return True if all active wheels are within tolerance of target
   */
  public boolean isAtSpeed() {
    if (m_targetVelocityRPS == 0.0) {
      return false;
    }

    boolean shooter1Ready = true;
    boolean shooter2Ready = true;
    boolean shooter3Ready = true;

    // Check each shooter based on active selection
    switch (m_activeShooters) {
      case ALL:
        shooter1Ready = isShooter1AtSpeed();
        shooter2Ready = isShooter2AtSpeed();
        shooter3Ready = isShooter3AtSpeed();
        break;
      case SHOOTER_1:
        shooter1Ready = isShooter1AtSpeed();
        shooter2Ready = true; // Ignore inactive
        shooter3Ready = true;
        break;
      case SHOOTER_2:
        shooter1Ready = true;
        shooter2Ready = isShooter2AtSpeed();
        shooter3Ready = true;
        break;
      case SHOOTER_3:
        shooter1Ready = true;
        shooter2Ready = true;
        shooter3Ready = isShooter3AtSpeed();
        break;
      case SHOOTERS_1_2:
        shooter1Ready = isShooter1AtSpeed();
        shooter2Ready = isShooter2AtSpeed();
        shooter3Ready = true;
        break;
      case SHOOTERS_2_3:
        shooter1Ready = true;
        shooter2Ready = isShooter2AtSpeed();
        shooter3Ready = isShooter3AtSpeed();
        break;
      case SHOOTERS_1_3:
        shooter1Ready = isShooter1AtSpeed();
        shooter2Ready = true;
        shooter3Ready = isShooter3AtSpeed();
        break;
    }

    return shooter1Ready && shooter2Ready && shooter3Ready;
  }

  private boolean isShooter1AtSpeed() {
    double topError = Math.abs(getShooter1TopVelocityRPS() - m_targetVelocityRPS);
    double bottomError = Math.abs(getShooter1BottomVelocityRPS() - m_targetVelocityRPS);
    return topError < VELOCITY_TOLERANCE_RPS && bottomError < VELOCITY_TOLERANCE_RPS;
  }

  private boolean isShooter2AtSpeed() {
    double topError = Math.abs(getShooter2TopVelocityRPS() - m_targetVelocityRPS);
    double bottomError = Math.abs(getShooter2BottomVelocityRPS() - m_targetVelocityRPS);
    return topError < VELOCITY_TOLERANCE_RPS && bottomError < VELOCITY_TOLERANCE_RPS;
  }

  private boolean isShooter3AtSpeed() {
    double topError = Math.abs(getShooter3TopVelocityRPS() - m_targetVelocityRPS);
    double bottomError = Math.abs(getShooter3BottomVelocityRPS() - m_targetVelocityRPS);
    return topError < VELOCITY_TOLERANCE_RPS && bottomError < VELOCITY_TOLERANCE_RPS;
  }

  /**
   * Checks if shooters are ready to shoot (at speed with current stable)
   * @return True if ready to shoot
   */
  public boolean isReady() {
    return isAtSpeed(); // Could add current checks here too
  }

  /**
   * Updates PID values from tunable numbers
   */
  private void updatePIDValues() {
    Slot0Configs pidConfigs = new Slot0Configs();
    pidConfigs.kP = m_kp.get();
    pidConfigs.kI = m_ki.get();
    pidConfigs.kD = m_kd.get();
    pidConfigs.kV = m_kv.get();
    pidConfigs.kS = m_ks.get();

    // Apply to all motors
    m_shooter1Top.getConfigurator().apply(pidConfigs);
    m_shooter1Bottom.getConfigurator().apply(pidConfigs);
    m_shooter2Top.getConfigurator().apply(pidConfigs);
    m_shooter2Bottom.getConfigurator().apply(pidConfigs);
    m_shooter3Top.getConfigurator().apply(pidConfigs);
    m_shooter3Bottom.getConfigurator().apply(pidConfigs);
  }

  /**
   * Sets the neutral mode for all motors
   * @param mode Brake or Coast
   */
  public void setNeutralMode(NeutralModeValue mode) {
    MotorOutputConfigs output = new MotorOutputConfigs();
    output.NeutralMode = mode;
    
    m_shooter1Top.getConfigurator().apply(output);
    m_shooter1Bottom.getConfigurator().apply(output);
    m_shooter2Top.getConfigurator().apply(output);
    m_shooter2Bottom.getConfigurator().apply(output);
    m_shooter3Top.getConfigurator().apply(output);
    m_shooter3Bottom.getConfigurator().apply(output);
  }
}