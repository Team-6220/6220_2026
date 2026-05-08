// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

/** Turret-specific hardware/constants used by IO and simulation. */

/** Add your docs here. */
public final class TurretConstants {
  public static final double kP = 0.01;
  public static final double kI = 0.01;
  public static final double kD = 0.01;

  /** Uses ANGULARVELOCITY because it can take any angular velocity unit */
  public static final AngularVelocity MAXIMUM_ANGULAR_VELOCITY = DegreesPerSecond.of(120);

  /** Uses ANGULARVELOCITY because it can take any angular velocity unit */
  public static final AngularAcceleration MAXIMUM_ANGULAR_ACCELERATION =
      DegreesPerSecondPerSecond.of(240);

  // Numeric convenience (radians/sec and radians/sec^2)
  public static final double MAX_VEL_RAD_PER_SEC = Math.toRadians(120.0);
  public static final double MAX_ACCEL_RAD_PER_SEC_S = Math.toRadians(240.0);

  // --------- Physical model ---------
  // DCMotor model used for simulation and feedforward. By default this uses a single NEO
  // (Neo 550 can be represented the same way). To change motor family, replace the factory
  // below (e.g. DCMotor.getCIMs(2) for CIMs, or DCMotor.getNEO(1) for a single NEO).
  public static final DCMotor TURRET_MOTOR = DCMotor.getNEO(1);

  // Gear reduction between motor and turret (motor rotations -> turret rotations)
  // Update to the real gearbox reduction for accurate simulation.
  public static final double TURRET_REDUCTION = 1.0; // TODO: set real reduction

  // Simple feedforward gains used by TurretSubsystem
  public static final double FF_kS = 0.0;
  public static final double FF_kV = 0.0;
  public static final double FF_kA = 0.0;

  // Motion profiling / motion magic settings (tunable)
  public static final double SPARK_MAX_SMARTMOTION_MAX_VEL_RAD_PER_SEC = 10.0;
  public static final double SPARK_MAX_SMARTMOTION_MAX_ACCEL_RAD_PER_SEC_S = 30.0;

  public static final double TALON_MOTION_MAGIC_MAX_VEL_RAD_PER_SEC = 10.0;
  public static final double TALON_MOTION_MAGIC_MAX_ACCEL_RAD_PER_SEC_S = 30.0;

  // Convenience feedforward object for use in simulation/subsystem
  public static final SimpleMotorFeedforward TURRET_FEEDFORWARD =
      new SimpleMotorFeedforward(FF_kS, FF_kV, FF_kA);
}
