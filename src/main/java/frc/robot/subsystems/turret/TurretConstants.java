// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

/** Add your docs here. */
public final class TurretConstants {
    public static final double kP = 0.01;
    public static final double kI = 0.01;
    public static final double kD = 0.01;

    /**Uses ANGULARVELOCITY because it can take any angular velocity unit  */
    public static final AngularVelocity MAXIMUM_ANGULAR_VELOCITY = DegreesPerSecond.of(120);
    /**Uses ANGULARVELOCITY because it can take any angular velocity unit  */
    public static final AngularAcceleration MAXIMUM_ANGULAR_ACCELERATION = DegreesPerSecondPerSecond.of(240);
}
