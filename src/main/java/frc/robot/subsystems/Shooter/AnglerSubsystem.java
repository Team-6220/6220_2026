// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TunableNumber;

public class AnglerSubsystem extends SubsystemBase {

  private static final int ANGLER_MOTOR_ID = 19;
  private static final int CURRENT_LIMIT = 30;

  private final SparkMax m_anglerMotor;
  private final SparkAbsoluteEncoder m_absoluteEncoder;
  private final SparkClosedLoopController m_closedLoopController;

  // Tunable PID for angler position control
  private final TunableNumber m_anglerKp = new TunableNumber("Angler/kP", 1.0);
  private final TunableNumber m_anglerKi = new TunableNumber("Angler/kI", 0.0);
  private final TunableNumber m_anglerKd = new TunableNumber("Angler/kD", 0.0);

  // Tunable target angle (degrees)
  private final TunableNumber m_targetAngle = new TunableNumber("Angler/TargetAngleDeg", 45.0);

  // Tolerance for determining if angler is at position (degrees)
  private static final double ANGLE_TOLERANCE_DEG = 2.0;

  public AnglerSubsystem() {
    m_anglerMotor = new SparkMax(ANGLER_MOTOR_ID, MotorType.kBrushless);
    m_absoluteEncoder = m_anglerMotor.getAbsoluteEncoder();
    m_closedLoopController = m_anglerMotor.getClosedLoopController();

    configureMotor();
  }

  private void configureMotor() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(CURRENT_LIMIT).idleMode(IdleMode.kBrake).inverted(false);

    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(m_anglerKp.get(), m_anglerKi.get(), m_anglerKd.get())
        .outputRange(-0.3, 0.3);

    m_anglerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Sets the angler speed manually.
   *
   * @param speed Speed from -1.0 to 1.0
   */
  public void setSpeed(double speed) {
    m_anglerMotor.set(speed);
  }

  /**
   * Moves the angler to the target angle using closed-loop position control.
   *
   * @param degrees Target angle in degrees
   */
  public void setAngle(double degrees) {
    m_closedLoopController.setReference(degrees / 360.0, SparkMax.ControlType.kPosition);
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
   * Gets the current angle from the absolute encoder in degrees.
   *
   * @return Current angle in degrees (0-360)
   */
  public double getCurrentAngleDeg() {
    return m_absoluteEncoder.getPosition() * 360.0;
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
  }
}
