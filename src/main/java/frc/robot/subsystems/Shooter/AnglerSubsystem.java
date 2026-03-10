// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TunableNumber;

public class AnglerSubsystem extends SubsystemBase {

  private static final int ANGLER_MOTOR_ID = 19;
  private static final int CURRENT_LIMIT = 30;

  // 15 motor rotations = 1 shaft rotation
  // Position conversion factor: 1/15 = output rotations per motor rotation
  // Then * 360 to get degrees
  private static final double GEAR_RATIO = 1/15.0;

  // Angle limits in degrees
  private static final double MIN_ANGLE_DEG = 0.0;
  private static final double MAX_ANGLE_DEG = 2.5;

  private final SparkMax m_anglerMotor;
  private final RelativeEncoder m_encoder;
  private final SparkClosedLoopController m_closedLoopController;

  // Tunable PID for angler position control
  private final TunableNumber m_anglerKp = new TunableNumber("Angler/kP", 0.02);
  private final TunableNumber m_anglerKi = new TunableNumber("Angler/kI", 0.0);
  private final TunableNumber m_anglerKd = new TunableNumber("Angler/kD", 0.0);

  // Tunable target angle (degrees)
  private final TunableNumber m_targetAngle = new TunableNumber("Angler/TargetAngleDeg", 45.0);

  // Tolerance for determining if angler is at position (degrees)
  private static final double ANGLE_TOLERANCE_DEG = 2.0;

  public AnglerSubsystem() {
    m_anglerMotor = new SparkMax(ANGLER_MOTOR_ID, MotorType.kBrushless);
    configureMotor();
    m_encoder = m_anglerMotor.getEncoder();
    m_closedLoopController = m_anglerMotor.getClosedLoopController();

  }

  private void configureMotor() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(CURRENT_LIMIT).idleMode(IdleMode.kBrake).inverted(false);
    config.inverted(true);
    // Configure encoder with gear ratio so position reads in degrees
    config.encoder
        .positionConversionFactor(GEAR_RATIO)
        .velocityConversionFactor(GEAR_RATIO);
    // Use the primary (relative) encoder for closed loop
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(m_anglerKp.get(), m_anglerKi.get(), m_anglerKd.get())
        .outputRange(-0.3, 0.3);

    m_anglerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Resets the encoder position to 0. Call this when the angler is at a known position (e.g. hard
   * stop or home position).
   */
  public void resetEncoder() {
    m_encoder.setPosition(0);
  }

  /**
   * Resets the encoder to a specific angle in degrees. Use this if your known position isn't 0.
   *
   * @param degrees The current angle in degrees to set the encoder to
   */
  public void resetEncoderTo(double degrees) {
    m_encoder.setPosition(degrees);
  }

  /**
   * Sets the angler speed manually, with soft limits enforced.
   *
   * @param speed Speed from -1.0 to 1.0
   */
  public void setSpeed(double speed) {
    double currentAngle = getCurrentAngleDeg();

    // Prevent moving past limits
    // if (currentAngle <= MIN_ANGLE_DEG && speed < 0) {
    //   m_anglerMotor.set(0);
    //   return;
    // }
    if (currentAngle >= MAX_ANGLE_DEG && speed > 0) {
      m_anglerMotor.set(0);
      return;
    }

    m_anglerMotor.set(speed);
  }

  /**
   * Moves the angler to the target angle using closed-loop position control. The angle is clamped
   * to the min/max limits. Units are in degrees since we set the position conversion factor.
   *
   * @param degrees Target angle in degrees
   */
  public void setAngle(double degrees) {
    double clampedDegrees = MathUtil.clamp(degrees, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
    m_closedLoopController.setReference(clampedDegrees, SparkMax.ControlType.kPosition);
  }

  /** Moves the angler to the tunable target angle. */
  public void goToTargetAngle() {
    setAngle(m_targetAngle.get());
  }

  /** Stops the angler motor. */
  public void stop() {
    m_anglerMotor.set(0);
  }

  /**
   * Gets the current angle from the relative encoder in degrees. Already accounts for the 15:1 gear
   * ratio via the position conversion factor.
   *
   * @return Current angle in degrees
   */
  public double getCurrentAngleDeg() {
    return m_encoder.getPosition();
  }

  /**
   * Checks if the angler is at the target angle within tolerance.
   *
   * @param targetDegrees The target angle in degrees to check against
   * @return True if at position
   */
  public boolean isAtAngle(double targetDegrees) {
    return Math.abs(getCurrentAngleDeg() - targetDegrees) < ANGLE_TOLERANCE_DEG;
  }

  /** Checks if the angler is at the tunable target angle. */
  public boolean isAtTargetAngle() {
    return isAtAngle(m_targetAngle.get());
  }

  public double getTargetAngle() {
    return m_targetAngle.get();
  }

  public double getMinAngleDeg() {
    return MIN_ANGLE_DEG;
  }

  public double getMaxAngleDeg() {
    return MAX_ANGLE_DEG;
  }

  @Override
  public void periodic() {
    // Update PID if tunable numbers changed
    if (m_anglerKp.hasChanged() || m_anglerKi.hasChanged() || m_anglerKd.hasChanged()) {
      configureMotor();
    }

    // Telemetry
    SmartDashboard.putNumber("Angler/CurrentAngleDeg", getCurrentAngleDeg());
    SmartDashboard.putNumber("Angler/TargetAngleDeg", m_targetAngle.get());
    SmartDashboard.putBoolean("Angler/AtTarget", isAtTargetAngle());
    SmartDashboard.putNumber("Angler/MotorOutput", m_anglerMotor.getAppliedOutput());
    SmartDashboard.putNumber("Angler/MotorCurrent", m_anglerMotor.getOutputCurrent());
    SmartDashboard.putNumber("Angler/MinAngleDeg", MIN_ANGLE_DEG);
    SmartDashboard.putNumber("Angler/MaxAngleDeg", MAX_ANGLE_DEG);
  }
}
