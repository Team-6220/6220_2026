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

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private static IntakeSubsystem INSTANCE = null;

  private final TunableNumber ArmKp =
      new TunableNumber("arm kP", IntakeConstants.armKp); // TODO: match/make to constant.java
  private final TunableNumber ArmKi = new TunableNumber("arm kI", IntakeConstants.armKi);
  private final TunableNumber ArmKd = new TunableNumber("arm kD", IntakeConstants.armKd);
  private final TunableNumber ArmKg = new TunableNumber("arm kG", IntakeConstants.armKg);
  private final TunableNumber ArmKv = new TunableNumber("arm kV", IntakeConstants.armKv);
  private final TunableNumber ArmKs = new TunableNumber("arm kS", IntakeConstants.armKs);
  private final TunableNumber ArmKa = new TunableNumber("arm kA", IntakeConstants.armKa);
  private final TunableNumber ArmIZone =
      new TunableNumber("arm izone", IntakeConstants.armIZone); // default 3
  private final TunableNumber ArmTolerance =
      new TunableNumber("arm tolerance", IntakeConstants.armTolerance); // default 1.5

  private final TunableNumber ArmMaxVel =
      new TunableNumber("arm max vel", IntakeConstants.armMaxVel);
  private final TunableNumber ArmMaxAccel =
      new TunableNumber("arm max accel", IntakeConstants.armMaxAccel);

  private final TunableNumber ArmIdleVoltage =
      new TunableNumber("lower intake idle voltage", IntakeConstants.armIdleVoltage);
  private final TunableNumber ArmVoltage =
      new TunableNumber("lower intake voltage", IntakeConstants.armVoltage);
  private double idleOutVolt = IntakeConstants.armIdleVoltage;
  private double intakeOutVolt = IntakeConstants.armVoltage;

  private final ProfiledPIDController m_Controller;
  private ArmFeedforward m_Feedforward;
  private TrapezoidProfile.Constraints m_Constraints;
  private double feedForwardOutput, PIDOutput;
  private double lastUpdate = 0;

  private String tableKey = "arm_";

  private final SparkMax beltMotor;
  private final SparkMaxConfig beltMotorConfig = new SparkMaxConfig();

  private final SparkMax armMotor;
  private final SparkMaxConfig armMotorConfig = new SparkMaxConfig();

  private final SparkMax rollerMotor;
  private final SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();

  private final RelativeEncoder armEncoder;

  private static final class IntakeConstants {
    // ids
    public static final int beltID = 0;
    public static final int armMotorID = 0;
    public static final int rollerMotorID = 0;

    // couldnt tell you tbh (claude bs)
    public static final double INTAKESPEED = 0.75;
    public static final double ARMSPEED = 0.5;
    public static final int CURRENTLIMIT = 30;

    // this either fr
    public static final int stallLimit = 5;
    public static final int freeLimit = 20;

    // inverted
    public static final boolean beltInvert = false;
    public static final boolean armInvert = false;
    public static final boolean rollerInvert = false;

    // idlemode
    public static final IdleMode beltIdleMode = IdleMode.kBrake;
    public static final IdleMode armIdleMode = IdleMode.kBrake;
    public static final IdleMode rollerIdleMode = IdleMode.kBrake;

    // pid stuff idk
    // need to review no clue what any of these are
    public static final double armKp = 0.; // .4;
    public static final double armKi = 0.;
    public static final double armKd = 0.0;
    public static final double armKa = 0.0;
    public static final double armKg = 0.0; // .01;
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

  public IntakeSubsystem() {
    beltMotor = new SparkMax(IntakeConstants.beltID, MotorType.kBrushless);
    beltMotorConfig.inverted(IntakeConstants.beltInvert);
    beltMotorConfig.smartCurrentLimit(IntakeConstants.stallLimit, IntakeConstants.freeLimit);
    beltMotorConfig.idleMode(IntakeConstants.beltIdleMode);
    beltMotor.configure(
        beltMotorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);

    armMotor = new SparkMax(IntakeConstants.armMotorID, MotorType.kBrushless);
    armMotorConfig.inverted(IntakeConstants.armInvert);
    armMotorConfig.smartCurrentLimit(IntakeConstants.stallLimit, IntakeConstants.freeLimit);
    armMotorConfig.idleMode(IntakeConstants.armIdleMode);
    armMotor.configure(
        armMotorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);

    armEncoder = armMotor.getEncoder();

    rollerMotor = new SparkMax(IntakeConstants.rollerMotorID, MotorType.kBrushless);
    rollerMotorConfig.inverted(IntakeConstants.rollerInvert);
    rollerMotorConfig.smartCurrentLimit(IntakeConstants.stallLimit, IntakeConstants.freeLimit);
    rollerMotorConfig.idleMode(IntakeConstants.rollerIdleMode);
    rollerMotor.configure(
        rollerMotorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);

    m_Constraints = new TrapezoidProfile.Constraints(ArmMaxVel.get(), ArmMaxAccel.get());

    m_Controller = new ProfiledPIDController(ArmKp.get(), ArmKi.get(), ArmKd.get(), m_Constraints);

    m_Feedforward = new ArmFeedforward(ArmKs.get(), ArmKg.get(), ArmKv.get(), ArmKa.get());

    m_Controller.setIZone(ArmIZone.get());

    m_Controller.setTolerance(ArmTolerance.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber(tableKey + "Position", getPosition());
    // SmartDashboard.putNumber(tableKey + "rawPosition", lowerintakeEncoder.get());
    // SmartDashboard.putBoolean(tableKey + "atGoal", controllerAtGoal());
    // SmartDashboard.putNumber(tableKey + "armCurrent", armMotor.getOutputCurrent());
    // SmartDashboard.putNumber(tableKey + "armOutput", armMotor.getAppliedOutput());
    // SmartDashboard.putNumber(tableKey + "motorOutputManuel", 0);
    // SmartDashboard.putNumber(
    //     tableKey + "intakeMotorTemp", armMotor.getDeviceTemp().getValueAsDouble());
    // SmartDashboard.putNumber(
    //     tableKey + "intakeMotorTorqueCurrentDraw",
    //     armMotor.getTorqueCurrent().getValueAsDouble());
    // SmartDashboard.putNumber(
    //     tableKey + "intakeMotorSupplyCurrentDraw",
    //     armMotor.getSupplyCurrent().getValueAsDouble());
    // SmartDashboard.putNumber(
    //     tableKey + "intake current limit", lowerIntakeConfig.CurrentLimits.SupplyCurrentLimit);
    // SmartDashboard.putNumber(
    //     tableKey + "intakeMotorStatorCurrentLimit",
    //     armMotor.getStatorCurrent().getValueAsDouble());
    // SmartDashboard.putNumber(tableKey + "left temp", pivotMotorLeft.getMotorTemperature());
    // SmartDashboard.putNumber(tableKey + "right temp", pivotMotorRight.getMotorTemperature());

    if (ArmKp.hasChanged() || ArmKi.hasChanged() || ArmKd.hasChanged()) {
      m_Controller.setPID(ArmKp.get(), ArmKi.get(), ArmKd.get());
      System.out.println("new PID;P:" + ArmKp.get() + "I:" + ArmKi.get() + "D:" + ArmKd.get());
    }

    if (ArmKs.hasChanged() || ArmKg.hasChanged() || ArmKv.hasChanged()) {
      m_Feedforward =
          new ArmFeedforward(ArmKs.get(), ArmKg.get(), ArmKv.get(), IntakeConstants.armKa);
      System.out.println("new ff;s:" + ArmKs.get() + "g:" + ArmKg.get() + "v:" + ArmKv.get());
    }

    if (ArmMaxVel.hasChanged() || ArmMaxAccel.hasChanged()) {
      m_Constraints = new TrapezoidProfile.Constraints(ArmMaxVel.get(), ArmMaxAccel.get());
      m_Controller.setConstraints(m_Constraints);
      System.out.println(
          "new contraints;max vel:" + ArmMaxVel.get() + "max accel:" + ArmMaxAccel.get());
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

    if (goal > IntakeConstants.maxDegrees) {
      goal = IntakeConstants.maxDegrees;
    }
    if (goal < IntakeConstants.minDegrees) {
      goal = IntakeConstants.minDegrees;
    }

    m_Controller.setGoal(goal);
  }

  public void swingToGoal() {
    feedForwardOutput =
        m_Feedforward.calculate(
            (m_Controller.getSetpoint().position) * Math.PI / 180,
            m_Controller.getSetpoint().velocity * Math.PI / 180);

    lastUpdate = Timer.getFPGATimestamp();

    PIDOutput = m_Controller.calculate(getPosition());

    double calculatedOutput = PIDOutput + feedForwardOutput;

    SmartDashboard.putNumber(tableKey + "ffOut", feedForwardOutput);
    SmartDashboard.putNumber(tableKey + "pidOut", PIDOutput);
    SmartDashboard.putNumber(tableKey + "calculatedOutput", calculatedOutput);
    SmartDashboard.putNumber(tableKey + "setPoint", m_Controller.getSetpoint().position);
    SmartDashboard.putNumber(tableKey + "setPointVelocity", m_Controller.getSetpoint().velocity);
    SmartDashboard.putBoolean(tableKey + "atsetpoint", m_Controller.atSetpoint());
    SmartDashboard.putNumber(tableKey + "goal", m_Controller.getGoal().position);
    armMotor.setVoltage(calculatedOutput);
  }

  public void resetPID() {

    m_Controller.reset(getPosition());
  }

  /** Raw encoder value subtracted by the offset at zero */
  // TWEAK FOR GEAR RATIO ETC
  public double getPosition() {
    return (armEncoder
        .getPosition()); // (encoder value - offset) * gear ratio from shaft to encoder *360 to get
    // degrees
  }

  public void simpleDrive(double motorOutput) {
    // motorOutput *= 12;
    SmartDashboard.putNumber(tableKey + "motorOutputManuel", motorOutput);
    armMotor.setVoltage(motorOutput);
  }

  public boolean controllerAtGoal() {
    return m_Controller.atGoal();
  }

  /**
   * @deprecated USE setFront() INSTEAD (the intake for algae is the outtake for coarl and the
   *     intake for coral is the outtake for algae, too confusing)
   * @param spin
   * @param intake
   */
  public void spinFront(boolean spin, boolean intake) {
    if (ArmVoltage.hasChanged()) {
      intakeOutVolt = ArmVoltage.get();
    }
    if (spin && intake) {
      armMotor.setVoltage(intakeOutVolt);
    }
    if (spin && !intake) {
      armMotor.setVoltage(-intakeOutVolt);
    }
    if (!spin) {
      armMotor.set(0);
    }
  }

  public void maintainFront() {
    if (ArmIdleVoltage.hasChanged()) {
      idleOutVolt = ArmIdleVoltage.get();
    }
    armMotor.setVoltage(-idleOutVolt);
  }

  public void setMaxVel(double maxVel) {
    ArmMaxVel.setDefault(maxVel);
    SmartDashboard.putNumber("FrontIntake max vel", maxVel);
  }

  public void setMaxAccel(double maxAccel) {
    ArmMaxAccel.setDefault(maxAccel);
    SmartDashboard.putNumber("FrontIntake max accel", maxAccel);
  }

  public static synchronized IntakeSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new IntakeSubsystem();
    }
    return INSTANCE;
  }

  // driving in decimal percents
  public void simpleDriveArm(double motorOutput) {
    SmartDashboard.putNumber("arm output", motorOutput);
    armMotor.setVoltage(motorOutput);
  }

  public void simpleDriveRoller(double motorOutput) {
    SmartDashboard.putNumber("roller output", motorOutput);
    rollerMotor.setVoltage(motorOutput);
  }

  public void simpleDriveBelt(double motorOutput) {
    SmartDashboard.putNumber("belt output", motorOutput);
    beltMotor.setVoltage(motorOutput);
  }
}
