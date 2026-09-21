// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.bus.CANPort;

public class BeltSubsystem extends SubsystemBase {

  private static BeltSubsystem INSTANCE = null;

  private final TelemetryTable m_beltTelemetry =
    Telemetry.getTable("Belt");
  private final String tableKey = "belt_";

  private final SparkMax beltMotor;
  private final SparkMaxConfig beltMotorConfig = new SparkMaxConfig();

  private static final class BeltConstants {
    public static final int beltID = 42;

    public static final int stallLimit = 30;
    public static final int freeLimit = 30;

    public static final boolean beltInvert = false;
    public static final IdleMode beltIdleMode = IdleMode.kBrake;
  }

  public BeltSubsystem() {
    CANPort beltCANPort = CANPort.CAN_S2;
    beltMotor = new SparkMax(beltCANPort, BeltConstants.beltID, MotorType.kBrushless);
    beltMotorConfig.inverted(BeltConstants.beltInvert);
    beltMotorConfig.smartCurrentLimit(BeltConstants.stallLimit, BeltConstants.freeLimit);
    beltMotorConfig.idleMode(BeltConstants.beltIdleMode);
    beltMotor.configure(
        beltMotorConfig,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {}

  public void simpleDrive(double motorOutput) {
    double pct = Math.max(-1.0, Math.min(1.0, motorOutput));
    double volts = pct * 12.0;
    m_beltTelemetry.log(tableKey + "output pct", pct);
    m_beltTelemetry.log(tableKey + "output (V)", volts);
    beltMotor.setVoltage(volts);
  }

  public void stop() {
    beltMotor.setVoltage(0);
  }

  public static synchronized BeltSubsystem getInstance() {
    if (INSTANCE == null) INSTANCE = new BeltSubsystem();
    return INSTANCE;
  }
}
