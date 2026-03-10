// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeltSubsystem extends SubsystemBase {

  private static BeltSubsystem INSTANCE = null;

  private final String tableKey = "belt_";

  private final TalonFX beltMotor;
  private final TalonFXConfiguration beltMotorConfig = new TalonFXConfiguration();

  private static final class BeltConstants {
    public static final int beltID = 42;

    public static final int stallLimit = 30;
    public static final int freeLimit = 30;

    public static final boolean beltInvert = false;
    public static final IdleMode beltIdleMode = IdleMode.kBrake;
  }

  public BeltSubsystem() {
    beltMotor = new TalonFX(BeltConstants.beltID);

    beltMotorConfig.MotorOutput.Inverted =
        BeltConstants.beltInvert
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    beltMotorConfig.MotorOutput.NeutralMode =
        (BeltConstants.beltIdleMode == IdleMode.kBrake)
            ? NeutralModeValue.Brake
            : NeutralModeValue.Coast;

    beltMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    beltMotorConfig.CurrentLimits.SupplyCurrentLimit = BeltConstants.freeLimit;
    beltMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = BeltConstants.stallLimit;
    beltMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

    beltMotor.getConfigurator().apply(beltMotorConfig);
  }

  @Override
  public void periodic() {}

  public void simpleDrive(double motorOutput) {
    double pct = Math.max(-1.0, Math.min(1.0, motorOutput));
    double volts = pct * 12.0;
    SmartDashboard.putNumber(tableKey + "output pct", pct);
    SmartDashboard.putNumber(tableKey + "output (V)", volts);
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
