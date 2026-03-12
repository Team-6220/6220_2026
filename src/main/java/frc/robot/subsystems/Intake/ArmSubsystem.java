// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TunableNumber;

public class ArmSubsystem extends SubsystemBase {

  private static ArmSubsystem INSTANCE = null;

  // Non-gain tunables (kept as TunableNumber)
  private final TunableNumber ArmIZone = new TunableNumber("arm izone", ArmConstants.armIZone);
  private final TunableNumber ArmTolerance =
      new TunableNumber("arm tolerance", ArmConstants.armTolerance);
  private final TunableNumber ArmMaxVel = new TunableNumber("arm max vel", ArmConstants.armMaxVel);
  private final TunableNumber ArmMaxAccel =
      new TunableNumber("arm max accel", ArmConstants.armMaxAccel);
  private final TunableNumber ArmIdleVoltage =
      new TunableNumber("arm idle voltage", ArmConstants.armIdleVoltage);
  private final TunableNumber ArmVoltage =
      new TunableNumber("arm voltage", ArmConstants.armVoltage);

  // Tracked gain values — compared each loop to detect SmartDashboard edits
  private double lastKp, lastKi, lastKd, lastKs, lastKg, lastKv;

  private double idleOutVolt = ArmConstants.armIdleVoltage;
  private double intakeOutVolt = ArmConstants.armVoltage;

  private final ProfiledPIDController m_Controller;
  private ArmFeedforward m_Feedforward;
  private TrapezoidProfile.Constraints m_Constraints;
  private double feedForwardOutput, PIDOutput;
  private double lastUpdate = 0;

  // Telemetry keys
  private final String tableKey = "arm_";
  private final String tuningKey = "arm_tune_";

  private final SparkMax armMotor;
  private final SparkMaxConfig armMotorConfig = new SparkMaxConfig();
  private final RelativeEncoder armEncoder;

  private static final class ArmConstants {
    public static final int armMotorID = 18;

    public static final int stallLimit = 30;
    public static final int freeLimit = 30;

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
    armMotor = new SparkMax(ArmConstants.armMotorID, MotorType.kBrushless);
    armMotorConfig.inverted(ArmConstants.armInvert);
    armMotorConfig.smartCurrentLimit(ArmConstants.stallLimit, ArmConstants.freeLimit);
    armMotorConfig.idleMode(ArmConstants.armIdleMode);
    armMotor.configure(
        armMotorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

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

    // Seed tracked values so we can detect changes in periodic()
    lastKp = ArmConstants.armKp;
    lastKi = ArmConstants.armKi;
    lastKd = ArmConstants.armKd;
    lastKs = ArmConstants.armKs;
    lastKg = ArmConstants.armKg;
    lastKv = ArmConstants.armKv;

    // Push initial gain values to SmartDashboard — these will show as editable fields
    SmartDashboard.putNumber(tuningKey + "kP", lastKp);
    SmartDashboard.putNumber(tuningKey + "kI", lastKi);
    SmartDashboard.putNumber(tuningKey + "kD", lastKd);
    SmartDashboard.putNumber(tuningKey + "kS", lastKs);
    SmartDashboard.putNumber(tuningKey + "kG", lastKg);
    SmartDashboard.putNumber(tuningKey + "kV", lastKv);
  }

  @Override
  public void periodic() {
    // --- Live gain tuning via SmartDashboard ---
    // Read whatever is currently in the dashboard fields
    double sdKp = SmartDashboard.getNumber(tuningKey + "kP", lastKp);
    double sdKi = SmartDashboard.getNumber(tuningKey + "kI", lastKi);
    double sdKd = SmartDashboard.getNumber(tuningKey + "kD", lastKd);
    double sdKs = SmartDashboard.getNumber(tuningKey + "kS", lastKs);
    double sdKg = SmartDashboard.getNumber(tuningKey + "kG", lastKg);
    double sdKv = SmartDashboard.getNumber(tuningKey + "kV", lastKv);

    // If P, I, or D changed, push update to controller
    if (sdKp != lastKp || sdKi != lastKi || sdKd != lastKd) {
      m_Controller.setPID(sdKp, sdKi, sdKd);
      lastKp = sdKp;
      lastKi = sdKi;
      lastKd = sdKd;
      System.out.println("[Arm] PID updated -> P:" + sdKp + " I:" + sdKi + " D:" + sdKd);
    }

    // If Ks, Kg, or Kv changed, rebuild feedforward
    if (sdKs != lastKs || sdKg != lastKg || sdKv != lastKv) {
      m_Feedforward = new ArmFeedforward(sdKs, sdKg, sdKv, ArmConstants.armKa);
      lastKs = sdKs;
      lastKg = sdKg;
      lastKv = sdKv;
      System.out.println("[Arm] FF updated -> Ks:" + sdKs + " Kg:" + sdKg + " Kv:" + sdKv);
    }

    // --- Other TunableNumber updates ---
    if (ArmIZone.hasChanged()) {
      m_Controller.setIZone(ArmIZone.get());
    }

    if (ArmTolerance.hasChanged()) {
      m_Controller.setTolerance(ArmTolerance.get());
    }

    if (ArmMaxVel.hasChanged() || ArmMaxAccel.hasChanged()) {
      m_Constraints = new TrapezoidProfile.Constraints(ArmMaxVel.get(), ArmMaxAccel.get());
      m_Controller.setConstraints(m_Constraints);
    }

    // --- Telemetry ---

    // Position & controller state
    SmartDashboard.putNumber(tableKey + "position", getPosition());
    SmartDashboard.putNumber(tableKey + "goal", m_Controller.getGoal().position);
    SmartDashboard.putNumber(tableKey + "setpoint_pos", m_Controller.getSetpoint().position);
    SmartDashboard.putNumber(tableKey + "setpoint_vel", m_Controller.getSetpoint().velocity);
    SmartDashboard.putNumber(tableKey + "position_error", m_Controller.getPositionError());
    SmartDashboard.putNumber(tableKey + "velocity_error", m_Controller.getVelocityError());
    SmartDashboard.putBoolean(tableKey + "at_setpoint", m_Controller.atSetpoint());
    SmartDashboard.putBoolean(tableKey + "at_goal", m_Controller.atGoal());

    // Controller outputs
    SmartDashboard.putNumber(tableKey + "ff_output", feedForwardOutput);
    SmartDashboard.putNumber(tableKey + "pid_output", PIDOutput);
    SmartDashboard.putNumber(tableKey + "total_output", feedForwardOutput + PIDOutput);

    // Motor diagnostics
    SmartDashboard.putNumber(
        tableKey + "motor_output_volts", armMotor.getBusVoltage() * armMotor.getAppliedOutput());
    SmartDashboard.putNumber(tableKey + "motor_current_amps", armMotor.getOutputCurrent());
    SmartDashboard.putNumber(tableKey + "motor_temp_celsius", armMotor.getMotorTemperature());
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

    lastUpdate = Timer.getFPGATimestamp();
    PIDOutput = m_Controller.calculate(getPosition());

    // System.out.println("pos=" + getPosition() + " goal=" + m_Controller.getGoal().position 
    // + " pid=" + PIDOutput + " ff=" + feedForwardOutput);

    armMotor.setVoltage(PIDOutput + feedForwardOutput);

    
  }

   public void resetArmEncoder() {
    armEncoder.setPosition(0);
  }

  public void resetPID() {
    m_Controller.reset(getPosition());
  }

  public double getPosition() {
    return armEncoder.getPosition();
  }

  public boolean controllerAtGoal() {
    return m_Controller.atGoal();
  }

  public void simpleDriveArm(double motorOutput) {
    armMotor.setVoltage(motorOutput * 6);
  }

  public void maintain() {
    if (ArmIdleVoltage.hasChanged()) idleOutVolt = ArmIdleVoltage.get();
    armMotor.setVoltage(-idleOutVolt);
  }

  public void setMaxVel(double maxVel) {
    ArmMaxVel.setDefault(maxVel);
    SmartDashboard.putNumber(tableKey + "max vel", maxVel);
  }

  public void setMaxAccel(double maxAccel) {
    ArmMaxAccel.setDefault(maxAccel);
    SmartDashboard.putNumber(tableKey + "max accel", maxAccel);
  }

  public static synchronized ArmSubsystem getInstance() {
    if (INSTANCE == null) INSTANCE = new ArmSubsystem();
    return INSTANCE;
  }
}
