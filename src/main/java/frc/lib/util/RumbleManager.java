// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import org.wpilib.driverstation.GenericHID.RumbleType;
import org.wpilib.system.Timer;
import org.wpilib.driverstation.XboxController;

/** Add your docs here. */
public class RumbleManager {
  public static void rumble(XboxController driver, double delayTimeInSec) {
    driver.setRumble(RumbleType.LEFT_RUMBLE, 0.75);
    driver.setRumble(RumbleType.RIGHT_RUMBLE, 0.75);
    Timer.delay(delayTimeInSec);
    driver.setRumble(RumbleType.LEFT_RUMBLE, 0);
    driver.setRumble(RumbleType.RIGHT_RUMBLE, 0);
  }
}
