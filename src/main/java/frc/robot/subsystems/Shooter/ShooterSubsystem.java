// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TunableNumber;

public class ShooterSubsystem extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIO.Inputs inputs = new ShooterIO.Inputs();

  // Tunable PID and feedforward values
  private final TunableNumber shooterKPTN = new TunableNumber("Shooter/kP", 0.1);
  private final TunableNumber shooterKITN = new TunableNumber("Shooter/kI", 0.0);
  private final TunableNumber shooterKDTN = new TunableNumber("Shooter/kD", 0.0);
  private final TunableNumber shooterKVTN = new TunableNumber("Shooter/kV", 0.12);
  private final TunableNumber shooterKSTN = new TunableNumber("Shooter/kS", 0.0);
  private final TunableNumber shooterKATN = new TunableNumber("Shooter/kA", 0.0);

  // Tunable target velocity
  private final TunableNumber shooterTargetVelocityRPS_TN =
      new TunableNumber("Shooter/TargetVelocityRPS", 60.0);

  // Tolerance for determining if shooter is at speed (in RPS)
  private static final double VELOCITY_TOLERANCE_RPS = 2.0;

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
    io.applyPIDConfigs(
        shooterKPTN.get(),
        shooterKITN.get(),
        shooterKDTN.get(),
        shooterKVTN.get(),
        shooterKSTN.get(),
        shooterKATN.get());
  }

  /** Runs all shooter motors at the tunable target velocity. */
  public void runAtTargetVelocity() {
    double rps = shooterTargetVelocityRPS_TN.get();
    setVelocityRPS(rps);
  }

  /**
   * Sets all shooter motors to the given velocity in RPS.
   *
   * @param rps Target velocity in rotations per second
   */
  public void setVelocityRPS(double rps) {
    io.setVelocityRPS(rps);
  }

  /**
   * Sets all shooter motors to the given velocity in RPM.
   *
   * @param rpm Target velocity in rotations per minute
   */
  public void setVelocityRPM(double rpm) {
    setVelocityRPS(rpm / 60.0);
  }

  /** Stops all shooter motors. */
  public void stop() {
    io.stop();
  }

  /**
   * Runs all shooters at a percentage of max output (for testing).
   *
   * @param percent Percent output from -1.0 to 1.0
   */
  public void setPercentOutput(double percent) {
    io.setPercentOutput(percent);
  }

  /** Runs motors 9, 2, and 31 at percent output. */
  public void setBottomGroupPercent(double percent) {
    io.setBottomGroupPercent(percent);
  }

  /** Runs motors 41 and 1 at percent output. */
  public void setTopGroupPercent(double percent) {
    io.setTopGroupPercent(percent);
  }

  /** Stops motors 9, 2, and 31. */
  public void stopBottomGroup() {
    io.stopBottomGroup();
  }

  /** Stops motors 41 and 1. */
  public void stopTopGroup() {
    io.stopTopGroup();
  }

  // ========== Velocity Getters ==========

  public double getMotor41VelocityRPS() {
    return inputs.motor41VelocityRPS;
  }

  public double getMotor1VelocityRPS() {
    return inputs.motor1VelocityRPS;
  }

  public double getMotor9VelocityRPS() {
    return inputs.motor9VelocityRPS;
  }

  public double getMotor31VelocityRPS() {
    return inputs.motor31VelocityRPS;
  }

  public double getMotor2VelocityRPS() {
    return inputs.motor2VelocityRPS;
  }

  /**
   * Gets the average velocity of all five shooter motors.
   *
   * @return Average velocity in RPS
   */
  public double getAverageVelocityRPS() {
    return (getMotor41VelocityRPS()
            + getMotor1VelocityRPS()
            + getMotor9VelocityRPS()
            + getMotor31VelocityRPS()
            + getMotor2VelocityRPS())
        / 5.0;
  }

  /**
   * Checks if all shooter motors are within tolerance of the target velocity.
   *
   * @return True if all motors are at speed
   */
  public boolean isAtSpeed() {
    double target = shooterTargetVelocityRPS_TN.get();
    if (target == 0.0) {
      return false;
    }

    return Math.abs(getMotor41VelocityRPS() - target) < VELOCITY_TOLERANCE_RPS
        && Math.abs(getMotor1VelocityRPS() - target) < VELOCITY_TOLERANCE_RPS
        && Math.abs(getMotor9VelocityRPS() - target) < VELOCITY_TOLERANCE_RPS
        && Math.abs(getMotor31VelocityRPS() - target) < VELOCITY_TOLERANCE_RPS
        && Math.abs(getMotor2VelocityRPS() - target) < VELOCITY_TOLERANCE_RPS;
  }

  /**
   * Gets the current target velocity.
   *
   * @return Target velocity in RPS
   */
  public double getTargetVelocityRPS() {
    return shooterTargetVelocityRPS_TN.get();
  }

  /**
   * Sets the neutral mode for all motors.
   *
   * @param brake true for brake mode, false for coast mode
   */
  public void setNeutralMode(boolean brake) {
    io.setNeutralMode(brake);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // Update PID if tunable numbers changed
    if (shooterKPTN.hasChanged()
        || shooterKITN.hasChanged()
        || shooterKDTN.hasChanged()
        || shooterKVTN.hasChanged()
        || shooterKSTN.hasChanged()
        || shooterKATN.hasChanged()) {
      io.applyPIDConfigs(
          shooterKPTN.get(),
          shooterKITN.get(),
          shooterKDTN.get(),
          shooterKVTN.get(),
          shooterKSTN.get(),
          shooterKATN.get());
    }

    // Telemetry
    SmartDashboard.putNumber("Shooter/Motor41VelocityRPS", inputs.motor41VelocityRPS);
    SmartDashboard.putNumber("Shooter/Motor1VelocityRPS", inputs.motor1VelocityRPS);
    SmartDashboard.putNumber("Shooter/Motor9VelocityRPS", inputs.motor9VelocityRPS);
    SmartDashboard.putNumber("Shooter/Motor31VelocityRPS", inputs.motor31VelocityRPS);
    SmartDashboard.putNumber("Shooter/Motor2VelocityRPS", inputs.motor2VelocityRPS);
    SmartDashboard.putNumber("Shooter/AverageVelocityRPS", getAverageVelocityRPS());
    SmartDashboard.putNumber("Shooter/TargetVelocityRPS", shooterTargetVelocityRPS_TN.get());
    SmartDashboard.putBoolean("Shooter/AtSpeed", isAtSpeed());
    SmartDashboard.putNumber("Shooter/Motor41Voltage", inputs.motor41Voltage);
    SmartDashboard.putNumber("Shooter/Motor1Voltage", inputs.motor1Voltage);
    SmartDashboard.putNumber("Shooter/Motor9Voltage", inputs.motor9Voltage);
    SmartDashboard.putNumber("Shooter/Motor31Voltage", inputs.motor31Voltage);
    SmartDashboard.putNumber("Shooter/Motor2Voltage", inputs.motor2Voltage);
    SmartDashboard.putNumber("Shooter/Motor41Current", inputs.motor41Current);
    SmartDashboard.putNumber("Shooter/Motor1Current", inputs.motor1Current);
    SmartDashboard.putNumber("Shooter/Motor9Current", inputs.motor9Current);
    SmartDashboard.putNumber("Shooter/Motor31Current", inputs.motor31Current);
    SmartDashboard.putNumber("Shooter/Motor2Current", inputs.motor2Current);
  }
}
