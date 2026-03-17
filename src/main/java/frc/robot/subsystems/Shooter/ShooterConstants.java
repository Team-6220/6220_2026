// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import java.util.HashMap;

/** Add your docs here. */
public class ShooterConstants {
  //first number is distance
  //first number of array is rpm
  //second number of array is angler angle
  public static HashMap<Double, Double[]> rpmAngle =
      new HashMap<Double, Double[]>() {
        {
          put(1.0, new Double[] {2150.0, 6.47});
          put(1.2, new Double[] {2060.0, 5.85});
          put(1.4, new Double[] {2075.0, 5.78});
          put(1.6, new Double[] {2220.0, 8.42});
          put(1.8, new Double[] {2050.0, 12.14});
          put(2.0, new Double[] {2350.0, 13.11});
          put(2.2, new Double[] {2000.0, 7.8});
        }
      };

  public static final double topTESTrpm = 4000.0;

  public static final double bottomTESTrpm = 800;
}
