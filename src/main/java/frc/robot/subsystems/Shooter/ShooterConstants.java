// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import java.util.HashMap;

/** Add your docs here. */
public class ShooterConstants {
  // first number is distance
  // first number of array is rpm
  // second number of array is angler angle
  public static HashMap<Double, Double[]> rpmAngle =
      new HashMap<Double, Double[]>() {
        {
          put(-1.0, new Double[] {0.0, 0.0}); // Testing purposes
          put(0.0, new Double[] {1860.0, 0.0}); // Testing purposes
          put(1.0, new Double[] {2150.0, 6.47});
          put(1.2, new Double[] {2040.0, 5.85});
          put(1.4, new Double[] {2090.0, 6.30});
          put(1.6, new Double[] {2040.0, 8.0});
          put(1.8, new Double[] {2000.0, 10.0});
          put(2.0, new Double[] {1980.0, 12.61});
          put(2.2, new Double[] {2100.0, 9.8});
          put(2.4, new Double[] {0.0, });
        }
      };

  public static final double topTESTrpm = 1600.0;

  public static final double bottomTESTrpm = 800;

  // ========== First Shot Boost Configuration ==========
  /** Multiplier for first shot RPM */
  public static double FIRST_SHOT_BOOST_PERCENT = 1.15;

  /** RPM dip threshold to detect when shot has left the shooter (in RPM) */
  public static final double RPM_DIP_THRESHOLD = 300.0;
}
