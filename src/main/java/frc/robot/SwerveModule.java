package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.config.RobotConfig;
import frc.robot.subsystems.Drive.SwerveModuleIO;
import frc.robot.subsystems.Drive.SwerveModuleIO.SwerveModuleIOInputs;

public class SwerveModule {
  private final int moduleNumber;
  private final SwerveModuleIO io;
  private final SwerveModuleIOInputs inputs = new SwerveModuleIOInputs();
  private final SwerveModuleConstants config;

  private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
  private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

  private final SimpleMotorFeedforward driveFeedForward =
      new SimpleMotorFeedforward(
          RobotConfig.SWERVECONFIG.driveKS(),
          RobotConfig.SWERVECONFIG.driveKV(),
          RobotConfig.SWERVECONFIG.driveKA());

  public SwerveModule(int moduleNumber, SwerveModuleConstants config, SwerveModuleIO io) {
    this.io = io;
    this.config = config;
    this.moduleNumber = moduleNumber;
  }

  public void periodic() {
    io.updateInputs(inputs);
    io.setAnglePosition(moduleNumber);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
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

  public void setDesiredState(SwerveModuleState state, boolean isOpenLoop) {
    // optimize
    state.optimize(getState().angle);

    // angle control
    io.setAnglePosition(RevConfigs.CANCoderAngleToNeoEncoder(state.angle.getRotations()));

    // drive control
    if (isOpenLoop) {
      driveDutyCycle.Output = state.speedMetersPerSecond / RobotConfig.SWERVECONFIG.maxSpeed();
      io.setDriveControlDutyCycle(driveDutyCycle);
    } else {
      driveVelocity.Velocity =
          Conversions.MPSToRPS(
              state.speedMetersPerSecond, RobotConfig.SWERVECONFIG.wheelCircumference());
      driveVelocity.FeedForward = driveFeedForward.calculate(state.speedMetersPerSecond);
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
