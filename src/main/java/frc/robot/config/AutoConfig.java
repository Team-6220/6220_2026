package frc.robot.config;

import frc.lib.util.TunableNumber;

public final class AutoConfig {

  // -----------------------------
  // TRANSLATION PID (raw doubles)
  // -----------------------------
  public static final TunableNumber translationKP = new TunableNumber("auto/translation_kP", 2.25);

  public static final TunableNumber translationKI = new TunableNumber("auto/translation_kI", 0.05);

  public static final TunableNumber translationKD = new TunableNumber("auto/translation_kD", 0.0);

  // -----------------------------
  // ANGULAR PID (raw doubles)
  // -----------------------------
  public static final TunableNumber angularKP = new TunableNumber("auto/angular_kP", 0.45);

  public static final TunableNumber angularKI = new TunableNumber("auto/angular_kI", 0.0);

  public static final TunableNumber angularKIzone = new TunableNumber("auto/angular_kIzone", 0.05);

  public static final TunableNumber angularKD = new TunableNumber("auto/angular_kD", 0.05);

  // -----------------------------
  // TRANSLATION CONSTRAINTS
  // (stored as doubles, converted to units)
  // -----------------------------
  public static final TunableNumber translationMaxVelMps =
      new TunableNumber("auto/translationMaxVel_mps", 5.0);

  public static final TunableNumber translationMaxAccelMpsSq =
      new TunableNumber("auto/translationMaxAccel_mps2", 15.0);

  public static double translationMaxVel() {
    return translationMaxVelMps.get();
  }

  public static double translationMaxAccel() {
    return translationMaxAccelMpsSq.get();
  }

  // -----------------------------
  // ANGULAR CONSTRAINTS (DEGREES)
  // -----------------------------
  public static final TunableNumber angularMaxVelDeg =
      new TunableNumber("auto/angularMaxVel_degPerSec", 240);

  public static final TunableNumber angularMaxAccelDeg =
      new TunableNumber("auto/angularMaxAccel_degPerSec2", 480);

  public static final TunableNumber angularToleranceDeg =
      new TunableNumber("auto/angularTolerance_deg", 5);

  // Convert to radians for controllers
  public static double angularMaxVelRad() {
    return Math.toRadians(angularMaxVelDeg.get());
  }

  public static double angularMaxAccelRad() {
    return Math.toRadians(angularMaxAccelDeg.get());
  }

  public static double angularToleranceRad() {
    return Math.toRadians(angularToleranceDeg.get());
  }
}
