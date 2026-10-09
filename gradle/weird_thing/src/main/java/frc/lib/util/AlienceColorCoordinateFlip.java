// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import frc.robot.Constants;

/** Add your docs here. */
public class AlienceColorCoordinateFlip {
  public AlienceColorCoordinateFlip() {}

  public static double flip(double x) {
    return Constants.isRed.equals("red") ? (16.54 - x) : x;
  }

  public static double flipDegrees(double degrees) {
    return Constants.isRed.equals("red") ? (180 - degrees) : degrees;
  }
}
