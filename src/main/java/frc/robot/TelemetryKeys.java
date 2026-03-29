// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * Centralized constants for all telemetry key names used in SmartDashboard and Elastic dashboard.
 * This ensures consistency across the codebase and prevents typos.
 *
 * <p>Keys follow a hierarchical naming convention: "Category/Subcategory/Property"
 */
public class TelemetryKeys {

  // ===== DRIVE TELEMETRY =====
  public static final String DRIVE_ROBOT_POSE_X = "Drive/RobotPose/X";
  public static final String DRIVE_ROBOT_POSE_Y = "Drive/RobotPose/Y";
  public static final String DRIVE_ROBOT_HEADING = "Drive/RobotHeading";
  public static final String DRIVE_CHASSIS_VX = "Drive/ChassisVelocity/Vx";
  public static final String DRIVE_CHASSIS_VY = "Drive/ChassisVelocity/Vy";
  public static final String DRIVE_CHASSIS_OMEGA = "Drive/ChassisVelocity/Omega";

  // Module-specific (0=BR, 1=BL, 2=FR, 3=FL)
  public static final String DRIVE_MODULE_0_SPEED = "Drive/Module0/Speed";
  public static final String DRIVE_MODULE_0_ANGLE = "Drive/Module0/Angle";
  public static final String DRIVE_MODULE_1_SPEED = "Drive/Module1/Speed";
  public static final String DRIVE_MODULE_1_ANGLE = "Drive/Module1/Angle";
  public static final String DRIVE_MODULE_2_SPEED = "Drive/Module2/Speed";
  public static final String DRIVE_MODULE_2_ANGLE = "Drive/Module2/Angle";
  public static final String DRIVE_MODULE_3_SPEED = "Drive/Module3/Speed";
  public static final String DRIVE_MODULE_3_ANGLE = "Drive/Module3/Angle";

  // ===== SHOOTER TELEMETRY =====
  public static final String SHOOTER_TOP_RPM = "Shooter/TopMotors/RPM";
  public static final String SHOOTER_TOP_TARGET_RPM = "Shooter/TopMotors/TargetRPM";
  public static final String SHOOTER_TOP_ERROR = "Shooter/TopMotors/Error";
  public static final String SHOOTER_BOTTOM_RPM = "Shooter/BottomMotors/RPM";
  public static final String SHOOTER_BOTTOM_TARGET_RPM = "Shooter/BottomMotors/TargetRPM";
  public static final String SHOOTER_BOTTOM_ERROR = "Shooter/BottomMotors/Error";
  public static final String SHOOTER_STATE = "Shooter/State";
  public static final String SHOOTER_DISTANCE = "Shooter/Distance";
  public static final String SHOOTER_ALIGNED = "Shooter/Aligned";
  public static final String SHOOTER_AT_TARGET = "Shooter/AtTarget";

  // ===== ANGLER TELEMETRY =====
  public static final String ANGLER_ANGLE = "Angler/CurrentAngle";
  public static final String ANGLER_TARGET = "Angler/TargetAngle";
  public static final String ANGLER_ERROR = "Angler/Error";
  public static final String ANGLER_AT_TARGET = "Angler/AtTarget";

  // ===== ARM TELEMETRY =====
  public static final String ARM_POSITION = "Arm/Position";
  public static final String ARM_GOAL = "Arm/Goal";
  public static final String ARM_ERROR = "Arm/Error";
  public static final String ARM_AT_GOAL = "Arm/AtGoal";
  public static final String ARM_PID_OUTPUT = "Arm/PIDOutput";
  public static final String ARM_FF_OUTPUT = "Arm/FFOutput";

  // ===== INTAKE TELEMETRY =====
  public static final String BELT_SPEED = "Intake/Belt/Speed";
  public static final String BELT_CURRENT = "Intake/Belt/Current";
  public static final String ROLLER_SPEED = "Intake/Roller/Speed";
  public static final String ROLLER_CURRENT = "Intake/Roller/Current";

  // ===== VISION TELEMETRY =====
  public static final String VISION_HAS_TARGET = "Vision/HasTarget";
  public static final String VISION_TARGET_OFFSET_X = "Vision/TargetOffset/X";
  public static final String VISION_TARGET_OFFSET_Y = "Vision/TargetOffset/Y";
  public static final String VISION_TARGET_DISTANCE = "Vision/TargetDistance";
  public static final String VISION_FIDUCIAL_ID = "Vision/FiducialID";
  public static final String VISION_PIPELINE = "Vision/Pipeline";
  public static final String VISION_TX = "Vision_tx";
  public static final String VISION_TY = "Vision_ty";
  public static final String VISION_TAG_ID = "Vision_tagID";

  // ===== SYSTEM TELEMETRY =====
  public static final String SYSTEM_BATTERY_VOLTAGE = "System/BatteryVoltage";
  public static final String SYSTEM_CAN_BUS_LOAD = "System/CANBusLoad";
  public static final String SYSTEM_ENABLED = "System/Enabled";
  public static final String SYSTEM_MODE = "System/Mode";

  // ===== AUTONOMOUS TELEMETRY =====
  public static final String AUTO_SELECTED = "Auto/Selected";
  public static final String AUTO_TIME = "Auto/Time";

  // Private constructor to prevent instantiation
  private TelemetryKeys() {}
}
