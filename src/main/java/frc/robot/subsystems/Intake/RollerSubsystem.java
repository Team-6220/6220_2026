// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSubsystem extends SubsystemBase {

  private static RollerSubsystem INSTANCE = null;

  private final String tableKey = "roller_";

  private final TalonFX rollerMotor;
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  private static final class RollerConstants {
    public static final int rollerMotorID = 15;
    public static final String canBus = "rio"; // change to your CANivore bus name if applicable

    public static final double stallCurrentLimit = 40.0;
    public static final boolean rollerInvert = false;
    public static final NeutralModeValue rollerIdleMode = NeutralModeValue.Brake;
  }

  public RollerSubsystem() {
    rollerMotor = new TalonFX(RollerConstants.rollerMotorID, RollerConstants.canBus);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted =
        RollerConstants.rollerInvert
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = RollerConstants.rollerIdleMode;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = RollerConstants.stallCurrentLimit;

    rollerMotor.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
        tableKey + "output (V)", rollerMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber(
        tableKey + "output pct", rollerMotor.getDutyCycle().getValueAsDouble());
  }

  public void simpleDrive(double motorOutput) {
    double pct = Math.max(-1.0, Math.min(1.0, motorOutput));
    rollerMotor.setControl(voltageRequest.withOutput(pct * 8.0));
  }

  public void spin(boolean intake) {
    rollerMotor.setControl(voltageRequest.withOutput(intake ? 8.0 : -8.0));
  }

  public void stop() {
    rollerMotor.setControl(voltageRequest.withOutput(0.0));
  }

  public static synchronized RollerSubsystem getInstance() {
    if (INSTANCE == null) INSTANCE = new RollerSubsystem();
    return INSTANCE;
  }
}
