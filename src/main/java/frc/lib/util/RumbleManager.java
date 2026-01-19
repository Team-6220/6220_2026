// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

/** Add your docs here. */
public class RumbleManager {
  public static void rumble(XboxController driver, double delayTimeInSec) {
    driver.setRumble(RumbleType.kBothRumble, 0.75);
    Timer.delay(delayTimeInSec);
    driver.setRumble(RumbleType.kBothRumble, 0);
  }
}
