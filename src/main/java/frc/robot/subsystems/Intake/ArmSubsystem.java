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

  private final TunableNumber ArmKp = new TunableNumber("arm kP", ArmConstants.armKp);
  private final TunableNumber ArmKi = new TunableNumber("arm kI", ArmConstants.armKi);
  private final TunableNumber ArmKd = new TunableNumber("arm kD", ArmConstants.armKd);
  private final TunableNumber ArmKg = new TunableNumber("arm kG", ArmConstants.armKg);
  private final TunableNumber ArmKv = new TunableNumber("arm kV", ArmConstants.armKv);
  private final TunableNumber ArmKs = new TunableNumber("arm kS", ArmConstants.armKs);
  private final TunableNumber ArmKa = new TunableNumber("arm kA", ArmConstants.armKa);
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

  private double idleOutVolt = ArmConstants.armIdleVoltage;
  private double intakeOutVolt = ArmConstants.armVoltage;

  private final ProfiledPIDController m_Controller;
  private ArmFeedforward m_Feedforward;
  private TrapezoidProfile.Constraints m_Constraints;
  private double feedForwardOutput, PIDOutput;
  private double lastUpdate = 0;

  private final String tableKey = "arm_";

  private final SparkMax armMotor;
  private final SparkMaxConfig armMotorConfig = new SparkMaxConfig();
  private final RelativeEncoder armEncoder;

  private static final class ArmConstants {
    public static final int armMotorID = 18;

    public static final int stallLimit = 30;
    public static final int freeLimit = 30;

    public static final boolean armInvert = false;
    public static final IdleMode armIdleMode = IdleMode.kBrake;

    public static final double armKp = 0.;
    public static final double armKi = 0.;
    public static final double armKd = 0.0;
    public static final double armKa = 0.0;
    public static final double armKg = 0.0;
    public static final double armKv = 0.0;
    public static final double armKs = 0;
    public static final double armIZone = 5;
    public static final double armTolerance = 0.5;
    public static final double armMaxVel = 200;
    public static final double armMaxAccel = 800;

    public static final double armIdleVoltage = 0.0;
    public static final double armVoltage = 5;

    public static final double maxDegrees = 0;
    public static final double minDegrees = 0;
  }

  public ArmSubsystem() {
    armMotor = new SparkMax(ArmConstants.armMotorID, MotorType.kBrushless);
    armMotorConfig.inverted(ArmConstants.armInvert);
    armMotorConfig.smartCurrentLimit(ArmConstants.stallLimit, ArmConstants.freeLimit);
    armMotorConfig.idleMode(ArmConstants.armIdleMode);
    armMotor.configure(
        armMotorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);

    armEncoder = armMotor.getEncoder();

    m_Constraints = new TrapezoidProfile.Constraints(ArmMaxVel.get(), ArmMaxAccel.get());
    m_Controller = new ProfiledPIDController(ArmKp.get(), ArmKi.get(), ArmKd.get(), m_Constraints);
    m_Feedforward = new ArmFeedforward(ArmKs.get(), ArmKg.get(), ArmKv.get(), ArmKa.get());
    m_Controller.setIZone(ArmIZone.get());
    m_Controller.setTolerance(ArmTolerance.get());
  }

  @Override
  public void periodic() {
    if (ArmKp.hasChanged() || ArmKi.hasChanged() || ArmKd.hasChanged()) {
      m_Controller.setPID(ArmKp.get(), ArmKi.get(), ArmKd.get());
      System.out.println("new PID;P:" + ArmKp.get() + "I:" + ArmKi.get() + "D:" + ArmKd.get());
    }

    if (ArmKs.hasChanged() || ArmKg.hasChanged() || ArmKv.hasChanged()) {
      m_Feedforward = new ArmFeedforward(ArmKs.get(), ArmKg.get(), ArmKv.get(), ArmConstants.armKa);
      System.out.println("new ff;s:" + ArmKs.get() + "g:" + ArmKg.get() + "v:" + ArmKv.get());
    }

    if (ArmMaxVel.hasChanged() || ArmMaxAccel.hasChanged()) {
      m_Constraints = new TrapezoidProfile.Constraints(ArmMaxVel.get(), ArmMaxAccel.get());
      m_Controller.setConstraints(m_Constraints);
      System.out.println(
          "new constraints;max vel:" + ArmMaxVel.get() + "max accel:" + ArmMaxAccel.get());
    }

    if (ArmIZone.hasChanged()) {
      m_Controller.setIZone(ArmIZone.get());
    }

    if (ArmTolerance.hasChanged()) {
      m_Controller.setTolerance(ArmTolerance.get());
    }
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

    double calculatedOutput = PIDOutput + feedForwardOutput;

    SmartDashboard.putNumber(tableKey + "ffOut", feedForwardOutput);
    SmartDashboard.putNumber(tableKey + "pidOut", PIDOutput);
    SmartDashboard.putNumber(tableKey + "calculatedOutput", calculatedOutput);
    SmartDashboard.putNumber(tableKey + "setPoint", m_Controller.getSetpoint().position);
    SmartDashboard.putNumber(tableKey + "setPointVelocity", m_Controller.getSetpoint().velocity);
    SmartDashboard.putBoolean(tableKey + "atSetpoint", m_Controller.atSetpoint());
    SmartDashboard.putNumber(tableKey + "goal", m_Controller.getGoal().position);

    armMotor.setVoltage(calculatedOutput);
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

  public void simpleDrive(double motorOutput) {
    double pct = Math.max(-1.0, Math.min(1.0, motorOutput));
    double volts = pct * 12.0;
    SmartDashboard.putNumber(tableKey + "motorOutputManuel", pct);
    SmartDashboard.putNumber(tableKey + "output (V)", volts);
    armMotor.setVoltage(volts);
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
