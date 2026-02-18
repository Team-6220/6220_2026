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

  public static final RobotType TYPE = RobotType.COMPBOT;
  public static boolean TUNING_MODE = (TYPE != RobotType.COMPBOT);

  public static Optional<DriverStation.Alliance> ALLIANCE_COLOR = DriverStation.getAlliance();

  public static final Mass robotMass = Pound.of(140);
  public static final MomentOfInertia robotMOI = KilogramSquareMeters.of(4.563);

  public static String isRed = "N/A"; // FIXME: MAKE AUTO UPDATE ISRED

  public enum RobotType {
    COMPBOT,
    PRACTICEBOT,
    SIMBOT
  }

  /** Checks whether the correct robot is selected when deploying. */
  public static class CheckDeploy {
    public static void main(String... args) {
      if (TYPE == RobotType.SIMBOT) {
        System.err.println("Cannot deploy, invalid robot selected: " + TYPE);
        System.exit(1);
      }
    }
  }

  /** Checks that the default robot is selected and tuning mode is disabled. */
  public static class CheckPullRequest {
    public static void main(String... args) {
      if (TYPE != RobotType.COMPBOT || TUNING_MODE) {
        System.err.println("Do not merge, non-default constants are configured.");
        System.exit(1);
      }
    }
  }
}
