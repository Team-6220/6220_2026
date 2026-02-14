// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pound;

import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;

public final class Constants {

  public static boolean TUNING_MODE = true;

  public static Optional<DriverStation.Alliance> ALLIANCE_COLOR = DriverStation.getAlliance();

  public static final Mass robotMass = Pound.of(140);
  public static final MomentOfInertia robotMOI = KilogramSquareMeters.of(4.563);

  public static String isRed = "N/A"; // FIXME: MAKE AUTO UPDATE ISRED

  public static final class ClimberConstants {
      // Motor CAN IDs
      public static final int climberDriverLeftID = 16;  // Change to your actual CAN ID
      // TODO: Add right motor CAN ID when installed on full robot
      // public static final int climberDriverRightID = 11; // Change to your actual CAN ID
      
      // Motor inversions
      public static final boolean motorAInverted = false; // Adjust based on your setup
      // TODO: Add right motor inversion when installed on full robot
      // public static final boolean motorBInverted = true;  // Adjust based on your setup
      
      // Servo configuration
      public static final int servoPWMPort = 0; // Change to your actual PWM port (0-9)
      public static final double servoAngleDegrees = 90.0; // Angle in degrees when button is pressed
      
      // Climber height configuration
      // This coefficient converts encoder rotations to height (inches or meters)
      // Example: If 1 rotation = 2 inches of height, coefficient = 2.0
      // Adjust based on your mechanism's gear ratio and drum/pulley diameter
      public static final double climberHeightCoefficient = 1.76*3.142; // Change to match your mechanism
    }
}
