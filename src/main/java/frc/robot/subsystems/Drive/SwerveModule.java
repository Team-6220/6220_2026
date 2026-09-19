package frc.robot.subsystems.Drive;

import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Rotation2d;
// import org.wpilib.math.kinematics.Kinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.lib.util.TunableNumber;
import frc.robot.RevConfigs;
import frc.robot.subsystems.Drive.SwerveModuleIO.SwerveModuleIOInputs;

public class SwerveModule {
  private final int moduleNumber;
  private final SwerveModuleIO io;
  private final SwerveModuleIOInputs inputs = new SwerveModuleIOInputs();
  private final SwerveModuleConstants config;

  private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
  private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

  /** driveKS Tunable Number */
  private final TunableNumber driveKSTN =
      new TunableNumber("SwerveModule_kS", SwerveConstants.DRIVE_KS);

  /** driveKV Tunable Number */
  private final TunableNumber driveKVTN =
      new TunableNumber("SwerveModule_kV", SwerveConstants.DRIVE_KV);

  /** driveKA Tunable Number */
  private final TunableNumber driveKATN =
      new TunableNumber("SwerveModule_kA", SwerveConstants.DRIVE_KA);

  private final SimpleMotorFeedforward driveFeedForward =
      new SimpleMotorFeedforward(driveKSTN.get(), driveKVTN.get(), driveKATN.get());

  public SwerveModule(int moduleNumber, SwerveModuleConstants config, SwerveModuleIO io) {
    this.io = io;
    this.config = config;
    this.moduleNumber = moduleNumber;
  }

  public void periodic() {
    io.updateInputs(inputs);
    if (driveKSTN.hasChanged()) {
      driveFeedForward.setKs(driveKSTN.get());
    }
    if (driveKVTN.hasChanged()) {
      driveFeedForward.setKv(driveKVTN.get());
    }
    if (driveKATN.hasChanged()) {
      driveFeedForward.setKa(driveKATN.get());
    }
  }

  public SwerveModuleVelocity getVelocities() {
    return new SwerveModuleVelocity(
        inputs.driveVelocityMps,
        Rotation2d.fromRadians(inputs.anglePositionRad.baseUnitMagnitude()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        inputs.drivePositionMeters, new Rotation2d(inputs.anglePositionRad));
  }

  public Rotation2d getCANcoder() {
    return inputs.absoluteAngle;
  }

  public int getModuleNumber() {
    return moduleNumber;
  }

  public void setDesiredVelocities(SwerveModuleVelocity velocities, boolean isOpenLoop) {
    // optimize
    velocities.optimize(getVelocities().angle);

    // angle control
    io.setAnglePosition(RevConfigs.CANCoderAngleToNeoEncoder(velocities.angle.getRotations()));

    // drive control
    if (isOpenLoop) {
      driveDutyCycle.Output = velocities.velocity / SwerveConstants.maxSpeed();
      io.setDriveControlDutyCycle(driveDutyCycle);
    } else {
      driveVelocity.Velocity =
          Conversions.MPSToRPS(velocities.velocity, SwerveConstants.wheelCircumference());
      driveVelocity.FeedForward = driveFeedForward.calculate(velocities.velocity);
      io.setDriveControlVelocity(driveVelocity);
    }
  }

  // ** Points the module forward */
  public void resetToAbsolute() {
    io.resetToAbsolute(config.angleOffset);
  }

  public void stopDriving() {
    io.setDriveVoltage(Volts.of(0));
  }
}
