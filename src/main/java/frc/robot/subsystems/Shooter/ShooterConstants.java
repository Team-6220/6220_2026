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
          put(-1.0, new Double[] {0.0, 0.0, 0.0}); // Testing purposes
          put(0.0, new Double[] {1900.0, 0.0, 1.15}); // Testing purposes
          put(0.8, new Double[] {1900.0, 0.0, 1.19});
          put(1.0, new Double[] {2050.0, 2.6, 1.2});
          put(1.2, new Double[] {2040.0, 5.85, 1.21});
          put(1.4, new Double[] {2060.0, 6.00, 1.22});
          put(1.6, new Double[] {2040.0, 8.0, 1.23});
          put(1.8, new Double[] {2000.0, 10.0, 1.24});
          put(2.0, new Double[] {1980.0, 12.61, 1.25});
          put(2.2, new Double[] {2000.0, 13.4, 1.26});
          put(2.4, new Double[] {2020.0, 14.10, 1.27});
          put(2.6, new Double[] {2040.0, 15.43, 1.28});
          put(2.8, new Double[] {2060.0, 16.47, 1.29});
          put(3.0, new Double[] {2080.0, 17.37, 1.3});
          put(3.2, new Double[] {2100.0, 18.30, 1.31});
          put(3.4, new Double[] {2120.0, 19.4, 1.32});
          put(3.6, new Double[] {2140.0, 20.45, 1.33});
          put(3.8, new Double[] {2160.0, 21.49, 1.34});
          put(4.0, new Double[] {2200.0, 22.5, 1.30});
        }
      };

  public static final double topTESTrpm = 1600.0;

  public static final double bottomTESTrpm = 800;

  // ========== First Shot Boost Configuration ==========
  /** Multiplier for first shot RPM (e.g., 1.15 for a 15% boost). */
  public static double FIRST_SHOT_BOOST_MULTIPLIER = 1.25;

  /**
   * @deprecated Use {@link #FIRST_SHOT_BOOST_MULTIPLIER} instead.
   */
  @Deprecated public static double FIRST_SHOT_BOOST_PERCENT = FIRST_SHOT_BOOST_MULTIPLIER;

  /** RPM dip threshold to detect when shot has left the shooter (in RPM) */
  public static final double RPM_DIP_THRESHOLD = 300.0;
}
