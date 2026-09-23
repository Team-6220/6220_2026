// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.system.Timer;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.bus.CANPort;

import frc.lib.util.TunableHelper;
import org.wpilib.tunable.TunableDouble;

public class ArmSubsystem extends SubsystemBase {

  private static ArmSubsystem INSTANCE = null;

  private final TelemetryTable m_armtelemetry =
    Telemetry.getTable("Arm");

  // Non-gain tunables (kept as TunableDouble)
  private final TunableDouble ArmIZone = TunableHelper.addDouble("arm izone", ArmConstants.armIZone);
  private final TunableDouble ArmTolerance =
      TunableHelper.addDouble("arm tolerance", ArmConstants.armTolerance);
  private final TunableDouble ArmMaxVel = TunableHelper.addDouble("arm max vel", ArmConstants.armMaxVel);
  private final TunableDouble ArmMaxAccel =
      TunableHelper.addDouble("arm max accel", ArmConstants.armMaxAccel);
  private final TunableDouble ArmIdleVoltage =
      TunableHelper.addDouble("arm idle voltage", ArmConstants.armIdleVoltage);
  private final TunableDouble ArmVoltage =
      TunableHelper.addDouble("arm voltage", ArmConstants.armVoltage);

  // Gain tunables — editable on the dashboard, applied in periodic() when changed
  private final TunableDouble ArmKp = TunableHelper.addDouble("arm_tune_kP", ArmConstants.armKp);
  private final TunableDouble ArmKi = TunableHelper.addDouble("arm_tune_kI", ArmConstants.armKi);
  private final TunableDouble ArmKd = TunableHelper.addDouble("arm_tune_kD", ArmConstants.armKd);
  private final TunableDouble ArmKs = TunableHelper.addDouble("arm_tune_kS", ArmConstants.armKs);
  private final TunableDouble ArmKg = TunableHelper.addDouble("arm_tune_kG", ArmConstants.armKg);
  private final TunableDouble ArmKv = TunableHelper.addDouble("arm_tune_kV", ArmConstants.armKv);

  private double idleOutVolt = ArmConstants.armIdleVoltage;
  private double intakeOutVolt = ArmConstants.armVoltage;

  private final ProfiledPIDController m_Controller;
  private ArmFeedforward m_Feedforward;
  private TrapezoidProfile.Constraints m_Constraints;
  private double feedForwardOutput, PIDOutput;
  private double lastUpdate = 0;

  private final SparkMax armMotor;
  private final SparkMaxConfig armMotorConfig = new SparkMaxConfig();
  private final RelativeEncoder armEncoder;

  private static final class ArmConstants {
    public static final int armMotorID = 18;

    public static final int stallLimit = 40;
    public static final int freeLimit = 40;

    public static final boolean armInvert = false;
    public static final IdleMode armIdleMode = IdleMode.kBrake;

    public static final double armKp = 0.45;
    public static final double armKi = 0.1;
    public static final double armKd = 0.0;
    public static final double armKa = 0.0;
    public static final double armKg = 0.3;
    public static final double armKv = 0.2;
    public static final double armKs = 0.0;
    public static final double armIZone = 5;
    public static final double armTolerance = 0.5;
    public static final double armMaxVel = 200;
    public static final double armMaxAccel = 800;

    public static final double armIdleVoltage = 0.0;
    public static final double armVoltage = 5;

    public static final double maxDegrees = 1000;
    public static final double minDegrees = -10000;
  }

  public ArmSubsystem() {
    CANPort armCANPort = CANPort.CAN_S1;
    armMotor = new SparkMax(armCANPort, ArmConstants.armMotorID, MotorType.kBrushless);
    armMotorConfig.inverted(ArmConstants.armInvert);
    armMotorConfig.smartCurrentLimit(ArmConstants.stallLimit, ArmConstants.freeLimit);
    armMotorConfig.idleMode(ArmConstants.armIdleMode);
    armMotor.configure(
        armMotorConfig,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);

    armEncoder = armMotor.getEncoder();

    m_Constraints =
        new TrapezoidProfile.Constraints(ArmConstants.armMaxVel, ArmConstants.armMaxAccel);
    m_Controller =
        new ProfiledPIDController(
            ArmConstants.armKp, ArmConstants.armKi, ArmConstants.armKd, m_Constraints);
    m_Feedforward =
        new ArmFeedforward(
            ArmConstants.armKs, ArmConstants.armKg, ArmConstants.armKv, ArmConstants.armKa);
    m_Controller.setIZone(ArmIZone.get());
    m_Controller.setTolerance(ArmTolerance.get());
  }

  @Override
  public void periodic() {
    // --- Live gain tuning via Tunables ---
    // If P, I, or D changed, push update to controller
    if (TunableHelper.consumeChanged(ArmKp, ArmKi, ArmKd)) {
      m_Controller.setPID(ArmKp.get(), ArmKi.get(), ArmKd.get());
      System.out.println(
          "[Arm] PID updated -> P:" + ArmKp.get() + " I:" + ArmKi.get() + " D:" + ArmKd.get());
    }

    // If Ks, Kg, or Kv changed, rebuild feedforward
    if (TunableHelper.consumeChanged(ArmKs, ArmKg, ArmKv)) {
      m_Feedforward =
          new ArmFeedforward(ArmKs.get(), ArmKg.get(), ArmKv.get(), ArmConstants.armKa);
      System.out.println(
          "[Arm] FF updated -> Ks:" + ArmKs.get() + " Kg:" + ArmKg.get() + " Kv:" + ArmKv.get());
    }

    // --- Other tunable updates ---
    if (TunableHelper.consumeChanged(ArmIZone)) {
      m_Controller.setIZone(ArmIZone.get());
    }

    if (TunableHelper.consumeChanged(ArmTolerance)) {
      m_Controller.setTolerance(ArmTolerance.get());
    }

    if (TunableHelper.consumeChanged(ArmMaxVel, ArmMaxAccel)) {
      m_Constraints = new TrapezoidProfile.Constraints(ArmMaxVel.get(), ArmMaxAccel.get());
      m_Controller.setConstraints(m_Constraints);
    }

    // --- Telemetry ---

    // Position & controller state
    m_armtelemetry.log("position", getPosition());
    m_armtelemetry.log("goal", m_Controller.getGoal().position);
    m_armtelemetry.log("setpoint_pos", m_Controller.getSetpoint().position);
    m_armtelemetry.log("setpoint_vel", m_Controller.getSetpoint().velocity);
    m_armtelemetry.log("position_error", m_Controller.getPositionError());
    m_armtelemetry.log("velocity_error", m_Controller.getVelocityError());
    m_armtelemetry.log("at_setpoint", m_Controller.atSetpoint());
    m_armtelemetry.log("at_goal", m_Controller.atGoal());

    // Controller outputs
    m_armtelemetry.log("ff_output", feedForwardOutput);
    m_armtelemetry.log("pid_output", PIDOutput);
    m_armtelemetry.log("total_output", feedForwardOutput + PIDOutput);

    // Motor diagnostics
    m_armtelemetry.log(
        "motor_output_volts", armMotor.getBusVoltage().get() * armMotor.getAppliedOutput().get());
    m_armtelemetry.log("motor_current_amps", armMotor.getOutputCurrent());
    m_armtelemetry.log("motor_temp_celsius", armMotor.getMotorTemperature());
  }

  public void setGoal(double goal) {
    resetPID();
    if (goal > ArmConstants.maxDegrees) goal = ArmConstants.maxDegrees;
    if (goal < ArmConstants.minDegrees) goal = ArmConstants.minDegrees;
    m_Controller.setGoal(goal);
  }

  public void swingToGoal() {
    feedForwardOutput =
        m_Feedforward.calculate(
            m_Controller.getSetpoint().position * Math.PI / 180,
            m_Controller.getSetpoint().velocity * Math.PI / 180);

    lastUpdate = Timer.getTimestamp();
    PIDOutput = m_Controller.calculate(getPosition());

    // System.out.println("pos=" + getPosition() + " goal=" + m_Controller.getGoal().position
    // + " pid=" + PIDOutput + " ff=" + feedForwardOutput);

    armMotor.setVoltage(PIDOutput + feedForwardOutput);
  }

  public void resetEncoder() {
    armEncoder.setPosition(0);
  }

  public void resetPID() {
    m_Controller.reset(getPosition());
  }

  public double getPosition() {
    return armEncoder.getPosition().get();
  }

  public boolean controllerAtGoal() {
    return m_Controller.atGoal();
  }

  public void simpleDriveArm(double motorOutput) {
    armMotor.setVoltage(motorOutput * 8);
  }

  public void maintain() {
    if (TunableHelper.consumeChanged(ArmIdleVoltage)) idleOutVolt = ArmIdleVoltage.get();
    armMotor.setVoltage(-idleOutVolt);
  }

  public void setMaxVel(double maxVel) {
    ArmMaxVel.set(maxVel);
    m_armtelemetry.log("max vel", maxVel);
  }

  public void setMaxAccel(double maxAccel) {
    ArmMaxAccel.set(maxAccel);
    m_armtelemetry.log("max accel", maxAccel);
  }

  public void stopDriving() {
    armMotor.stopMotor();
  }

  public static synchronized ArmSubsystem getInstance() {
    if (INSTANCE == null) INSTANCE = new ArmSubsystem();
    return INSTANCE;
  }
}
