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
          put(1.0, new Double[] {2150.0, 6.28});
          put(1.1, new Double[] {2000.0, 13.0});
          put(1.7, new Double[] {});
        }
      };

  public static final double topTESTrpm = 4000.0;

  public static final double bottomTESTrpm = 800;
}
