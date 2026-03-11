// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AnglerSubsystem extends SubsystemBase {

  private static final int ANGLER_MOTOR_ID = 19;
  private static final int CURRENT_LIMIT = 30;
  private static AnglerSubsystem INSTANCE = null;

  private final SparkMax m_anglerMotor;

  public AnglerSubsystem() {
    m_anglerMotor = new SparkMax(ANGLER_MOTOR_ID, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(CURRENT_LIMIT).idleMode(IdleMode.kBrake).inverted(false);

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

  /** Stops the angler motor. */
  public void stop() {
    m_anglerMotor.set(0);
  }

  @Override
  public void periodic() {}

  public static AnglerSubsystem getInstance() {
    if (INSTANCE == null) INSTANCE = new AnglerSubsystem();
    return INSTANCE;
  }
}
