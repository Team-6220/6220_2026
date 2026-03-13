// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import java.util.HashMap;

/** Add your docs here. */
public class ShooterConstants {
  public static HashMap<Double, Double[]> rpmAngle =
      new HashMap<Double, Double[]>() {
        {
          put(0.1, new Double[] {1000.0, 13.0});
          put(0.2, new Double[] {4000.0, 13.0});
        }
      };

  public static final double topTESTrpm = 4000.0;

  public static final double bottomTESTrpm = 800;
}
