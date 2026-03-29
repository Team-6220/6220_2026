// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Centralized telemetry publishing utility for Elastic dashboard integration.
 *
 * <p>This class provides a unified interface for publishing robot telemetry to SmartDashboard,
 * which is automatically ingested by the Elastic dashboard via NetworkTables. By centralizing
 * telemetry calls here, we ensure consistency and make it easy to adjust publishing logic without
 * modifying subsystem code.
 *
 * <p>Usage: Call static methods like {@code ElasticTelemetry.publishRobotPose(pose)} from subsystem
 * periodic() methods.
 */
public class ElasticTelemetry {

  private static ElasticTelemetry instance;

  private ElasticTelemetry() {
    // Private constructor for singleton pattern
  }

  /**
   * Get or create the singleton instance.
   *
   * @return the ElasticTelemetry singleton
   */
  public static synchronized ElasticTelemetry getInstance() {
    if (instance == null) {
      instance = new ElasticTelemetry();
    }
    return instance;
  }

  // ===== GENERIC TELEMETRY METHODS =====

  /**
   * Publish a numeric value to SmartDashboard (and Elastic via NetworkTables).
   *
   * @param key the telemetry key (use constants from TelemetryKeys)
   * @param value the numeric value
   */
  public static void publishNumber(String key, double value) {
    SmartDashboard.putNumber(key, value);
  }

  /**
   * Publish a boolean value to SmartDashboard.
   *
   * @param key the telemetry key
   * @param value the boolean value
   */
  public static void publishBoolean(String key, boolean value) {
    SmartDashboard.putBoolean(key, value);
  }

  /**
   * Publish a string value to SmartDashboard.
   *
   * @param key the telemetry key
   * @param value the string value
   */
  public static void publishString(String key, String value) {
    SmartDashboard.putString(key, value);
  }

  // ===== ROBOT-SPECIFIC CONVENIENCE METHODS =====

  /**
   * Publish robot pose (x, y, heading) to SmartDashboard.
   *
   * @param pose the robot's 2D pose
   */
  public static void publishRobotPose(Pose2d pose) {
    publishNumber(TelemetryKeys.DRIVE_ROBOT_POSE_X, pose.getX());
    publishNumber(TelemetryKeys.DRIVE_ROBOT_POSE_Y, pose.getY());
    publishNumber(TelemetryKeys.DRIVE_ROBOT_HEADING, pose.getRotation().getDegrees());
  }

  /**
   * Publish chassis velocity (vx, vy, omega).
   *
   * @param vx velocity in X direction (m/s)
   * @param vy velocity in Y direction (m/s)
   * @param omega rotational velocity (rad/s)
   */
  public static void publishChassisVelocity(double vx, double vy, double omega) {
    publishNumber(TelemetryKeys.DRIVE_CHASSIS_VX, vx);
    publishNumber(TelemetryKeys.DRIVE_CHASSIS_VY, vy);
    publishNumber(TelemetryKeys.DRIVE_CHASSIS_OMEGA, omega);
  }

  /**
   * Publish a swerve module's state (speed and angle).
   *
   * @param moduleIndex the module index (0-3)
   * @param state the module's desired/actual state
   */
  public static void publishModuleState(int moduleIndex, SwerveModuleState state) {
    String speedKey = null;
    String angleKey = null;

    switch (moduleIndex) {
      case 0:
        speedKey = TelemetryKeys.DRIVE_MODULE_0_SPEED;
        angleKey = TelemetryKeys.DRIVE_MODULE_0_ANGLE;
        break;
      case 1:
        speedKey = TelemetryKeys.DRIVE_MODULE_1_SPEED;
        angleKey = TelemetryKeys.DRIVE_MODULE_1_ANGLE;
        break;
      case 2:
        speedKey = TelemetryKeys.DRIVE_MODULE_2_SPEED;
        angleKey = TelemetryKeys.DRIVE_MODULE_2_ANGLE;
        break;
      case 3:
        speedKey = TelemetryKeys.DRIVE_MODULE_3_SPEED;
        angleKey = TelemetryKeys.DRIVE_MODULE_3_ANGLE;
        break;
      default:
        throw new IllegalArgumentException("Invalid module index: " + moduleIndex);
    }

    publishNumber(speedKey, state.speedMetersPerSecond);
    publishNumber(angleKey, state.angle.getDegrees());
  }

  /**
   * Publish shooter telemetry (RPM values, target, and distance).
   *
   * @param topRPM actual top motor RPM
   * @param topTargetRPM target top motor RPM
   * @param bottomRPM actual bottom motor RPM
   * @param bottomTargetRPM target bottom motor RPM
   * @param distance distance to target (meters)
   */
  public static void publishShooterState(
      double topRPM,
      double topTargetRPM,
      double bottomRPM,
      double bottomTargetRPM,
      double distance) {
    publishNumber(TelemetryKeys.SHOOTER_TOP_RPM, topRPM);
    publishNumber(TelemetryKeys.SHOOTER_TOP_TARGET_RPM, topTargetRPM);
    publishNumber(TelemetryKeys.SHOOTER_TOP_ERROR, Math.abs(topRPM - topTargetRPM));
    publishNumber(TelemetryKeys.SHOOTER_BOTTOM_RPM, bottomRPM);
    publishNumber(TelemetryKeys.SHOOTER_BOTTOM_TARGET_RPM, bottomTargetRPM);
    publishNumber(TelemetryKeys.SHOOTER_BOTTOM_ERROR, Math.abs(bottomRPM - bottomTargetRPM));
    publishNumber(TelemetryKeys.SHOOTER_DISTANCE, distance);
  }

  /**
   * Publish shooter readiness state.
   *
   * @param isReady true if shooter is at target speed
   * @param isAligned true if angler is aligned
   */
  public static void publishShooterReady(boolean isReady, boolean isAligned) {
    publishBoolean(TelemetryKeys.SHOOTER_AT_TARGET, isReady);
    publishBoolean(TelemetryKeys.SHOOTER_ALIGNED, isAligned);
  }

  /**
   * Publish angler telemetry.
   *
   * @param currentAngle current angle (degrees)
   * @param targetAngle target angle (degrees)
   * @param atTarget true if at target angle
   */
  public static void publishAnglerState(double currentAngle, double targetAngle, boolean atTarget) {
    publishNumber(TelemetryKeys.ANGLER_ANGLE, currentAngle);
    publishNumber(TelemetryKeys.ANGLER_TARGET, targetAngle);
    publishNumber(TelemetryKeys.ANGLER_ERROR, Math.abs(currentAngle - targetAngle));
    publishBoolean(TelemetryKeys.ANGLER_AT_TARGET, atTarget);
  }

  /**
   * Publish arm telemetry.
   *
   * @param position current arm position
   * @param goal goal arm position
   * @param atGoal true if at goal
   * @param pidOutput PID controller output (for debugging)
   * @param ffOutput feedforward output (for debugging)
   */
  public static void publishArmState(
      double position, double goal, boolean atGoal, double pidOutput, double ffOutput) {
    publishNumber(TelemetryKeys.ARM_POSITION, position);
    publishNumber(TelemetryKeys.ARM_GOAL, goal);
    publishNumber(TelemetryKeys.ARM_ERROR, Math.abs(position - goal));
    publishBoolean(TelemetryKeys.ARM_AT_GOAL, atGoal);
    publishNumber(TelemetryKeys.ARM_PID_OUTPUT, pidOutput);
    publishNumber(TelemetryKeys.ARM_FF_OUTPUT, ffOutput);
  }

  /**
   * Publish intake motor telemetry.
   *
   * @param beltSpeed belt motor speed (0-1)
   * @param beltCurrent belt motor current (amps)
   * @param rollerSpeed roller motor speed (0-1)
   * @param rollerCurrent roller motor current (amps)
   */
  public static void publishIntakeMotors(
      double beltSpeed, double beltCurrent, double rollerSpeed, double rollerCurrent) {
    publishNumber(TelemetryKeys.BELT_SPEED, beltSpeed);
    publishNumber(TelemetryKeys.BELT_CURRENT, beltCurrent);
    publishNumber(TelemetryKeys.ROLLER_SPEED, rollerSpeed);
    publishNumber(TelemetryKeys.ROLLER_CURRENT, rollerCurrent);
  }

  /**
   * Publish vision (Limelight) telemetry.
   *
   * @param hasTarget true if target is visible
   * @param targetOffsetX horizontal offset to target (degrees)
   * @param targetOffsetY vertical offset to target (degrees)
   * @param distance estimated distance to target (meters)
   * @param fiducialID AprilTag ID
   * @param pipeline current pipeline index
   */
  public static void publishVisionState(
      boolean hasTarget,
      double targetOffsetX,
      double targetOffsetY,
      double distance,
      double fiducialID,
      double pipeline) {
    publishBoolean(TelemetryKeys.VISION_HAS_TARGET, hasTarget);
    publishNumber(TelemetryKeys.VISION_TARGET_OFFSET_X, targetOffsetX);
    publishNumber(TelemetryKeys.VISION_TARGET_OFFSET_Y, targetOffsetY);
    publishNumber(TelemetryKeys.VISION_TARGET_DISTANCE, distance);
    publishNumber(TelemetryKeys.VISION_FIDUCIAL_ID, fiducialID);
    publishNumber(TelemetryKeys.VISION_PIPELINE, pipeline);
  }

  /**
   * Publish system telemetry.
   *
   * @param batteryVoltage robot battery voltage
   * @param canBusLoad CAN bus utilization (percent)
   */
  public static void publishSystemState(double batteryVoltage, double canBusLoad) {
    publishNumber(TelemetryKeys.SYSTEM_BATTERY_VOLTAGE, batteryVoltage);
    publishNumber(TelemetryKeys.SYSTEM_CAN_BUS_LOAD, canBusLoad);
  }
}
