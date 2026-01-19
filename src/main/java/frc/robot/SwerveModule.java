package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.config.RobotConfig;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d angleOffset;

  // private TalonFX mAngleMotor;
  private SparkMax mAngleMotor;
  private SparkClosedLoopController mAngleController;
  private RelativeEncoder mNeoAngleEncoder;
  private TalonFX mDriveMotor;
  private CANcoder angleEncoder;
  SparkMaxConfig angleConfig = new SparkMaxConfig();

  private double overallDesiredModuleState;

  private final SimpleMotorFeedforward driveFeedForward =
      new SimpleMotorFeedforward(
          SwerveConstants.driveKS, SwerveConstants.driveKV, SwerveConstants.driveKA);

  /* drive motor control requests */
  private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
  private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

  /* angle motor control requests */
  // private final PositionVoltage anglePosition = new PositionVoltage(0);

  @SuppressWarnings("removal")
  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    this.angleOffset = moduleConstants.angleOffset;
    SmartDashboard.putString("Mod: " + moduleNumber, angleOffset.toString());

    /* Angle Encoder Config */
    angleEncoder = new CANcoder(moduleConstants.cancoderID);
    angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

    /* Angle Motor Config */
    mAngleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    angleConfig
        .inverted(SwerveConstants.angleMotorInvert)
        .idleMode(SwerveConstants.angleNeutralMode)
        .smartCurrentLimit(SwerveConstants.angleCurrentLimit);

    // angleConfig.encoder
    //     .countsPerRevolution(42);

    /* Angle Motor PID Config */
    mAngleController = mAngleMotor.getClosedLoopController();

    angleConfig
        .closedLoop
        .pid(SwerveConstants.angleKP, SwerveConstants.angleKI, SwerveConstants.angleKD)
        .positionWrappingEnabled(
            true) // wraps the numbers around when it's too big. ex if the limits are 0 and 100, it
        // will "wrap" back to 0 after it exceeds 100, vise versa
        .positionWrappingMinInput(0)
        .positionWrappingMaxInput(RevConfigs.CANCoderAngleToNeoEncoder(1));

    /* Angle Motor Encoder Config */
    mNeoAngleEncoder = mAngleMotor.getEncoder();

    mAngleMotor.configure(
        angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    resetToAbsolute();

    /* Drive Motor Config */
    mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
    mDriveMotor
        .getConfigurator()
        .apply(
            Robot.ctreConfigs
                .swerveDriveFXConfig); // motor inverted, current limits, etc. editable in
    // constants.java. CTREConfigs.java is just a holder to
    // organize the values
    mDriveMotor.getConfigurator().setPosition(0.0);

    // mDriveMotor.setNeutralMode(SwerveConstants.driveNeutralMode);
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
    desiredState.optimize(getState().angle);
    mAngleController.setSetpoint(
        RevConfigs.CANCoderAngleToNeoEncoder(desiredState.angle.getRotations()),
        ControlType.kPosition);
    overallDesiredModuleState = desiredState.angle.getDegrees();
    SmartDashboard.putNumber("Desired position", (desiredState.angle.getDegrees()));
    setSpeed(desiredState, isOpenLoop);
  }

  public double getDesiredState() {
    return overallDesiredModuleState;
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    SmartDashboard.putBoolean("mod" + moduleNumber + "isopenloop", isOpenLoop);
    if (isOpenLoop) {
      driveDutyCycle.Output =
          desiredState.speedMetersPerSecond / RobotConfig.SWERVECONFIG.maxSpeed();
      mDriveMotor.setControl(driveDutyCycle);
      SmartDashboard.putNumber("Mod " + moduleNumber + "drivedutycycle", driveDutyCycle.Output);
    } else {
      driveVelocity.Velocity =
          Conversions.MPSToRPS(
              desiredState.speedMetersPerSecond, RobotConfig.SWERVECONFIG.wheelCircumference());
      driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
      mDriveMotor.setControl(driveVelocity);
    }
  }

  public void stopDriving() {
    mDriveMotor.set(0);
  }

  public Rotation2d getCANcoder() {
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
  }

  // ** Points the module forward */
  public void resetToAbsolute() {
    double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
    SmartDashboard.putNumber("absolutePosition", absolutePosition);
    mNeoAngleEncoder.setPosition(RevConfigs.CANCoderAngleToNeoEncoder(absolutePosition));
    SmartDashboard.putNumber("absolutePosition degress", mNeoAngleEncoder.getPosition() * 360);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        Conversions.RPSToMPS(
            mDriveMotor.getVelocity().getValueAsDouble(),
            RobotConfig.SWERVECONFIG.wheelCircumference()),
        Rotation2d.fromRotations(
            RevConfigs.NeoEncoderAngleToCANCoder(mNeoAngleEncoder.getPosition())));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        Conversions.rotationsToMeters(
            mDriveMotor.getPosition().getValueAsDouble(),
            RobotConfig.SWERVECONFIG.wheelCircumference()),
        Rotation2d.fromRotations(
            RevConfigs.NeoEncoderAngleToCANCoder(mNeoAngleEncoder.getPosition())));
  }
}
