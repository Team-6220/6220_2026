// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSubsystem extends SubsystemBase {

  private static RollerSubsystem INSTANCE = null;

  private final String tableKey = "roller_";

  private final SparkMax rollerMotor;
  private final SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();

  private static final class RollerConstants {
    public static final int rollerMotorID = 15;

    public static final int stallLimit = 30;
    public static final int freeLimit = 30;

    public static final boolean rollerInvert = false;
    public static final IdleMode rollerIdleMode = IdleMode.kBrake;
  }

  public RollerSubsystem() {
    rollerMotor = new SparkMax(RollerConstants.rollerMotorID, MotorType.kBrushless);
    rollerMotorConfig.inverted(RollerConstants.rollerInvert);
    rollerMotorConfig.smartCurrentLimit(RollerConstants.stallLimit, RollerConstants.freeLimit);
    rollerMotorConfig.idleMode(RollerConstants.rollerIdleMode);
    rollerMotor.configure(
        rollerMotorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {}

  public void simpleDrive(double motorOutput) {
    double pct = Math.max(-1.0, Math.min(1.0, motorOutput));
    double volts = pct * 8.0;
    SmartDashboard.putNumber(tableKey + "output pct", pct);
    SmartDashboard.putNumber(tableKey + "output (V)", volts);
    rollerMotor.setVoltage(volts);
  }

  public void spin(boolean intake) {
    rollerMotor.setVoltage(intake ? 5.0 : -5.0);
  }

  public void stop() {
    rollerMotor.set(0);
  }

  public static synchronized RollerSubsystem getInstance() {
    if (INSTANCE == null) INSTANCE = new RollerSubsystem();
    return INSTANCE;
  }
}
