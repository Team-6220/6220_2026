package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.lib.util.TunableNumber;

public final class AutoConstants {
  // FIXME: The below constants are used in the example auto, and must be
  // tuned to specific robot

  public static final double translation_kP =
      2.25; // Used to be 7 but that was never used in autobuilder(and maybe way too high)
  public static final double translation_kI = 0.05;
  public static final double translation_kD = 0;
  public static final double angular_kP =
      0.45; // Used to be 6 but that was never used in autobuilder(and maybe way too high)
  public static final double angular_kI = 0;
  public static final double angular_kIzone = 0.05;
  public static final double angular_kD = 0.05;

  public static final LinearVelocity translationMaxVelocityMps = MetersPerSecond.of(5);
  public static final LinearAcceleration translationMaxAcceleratMpsSq =
      MetersPerSecondPerSecond.of(15);
  public static final AngularVelocity maxAngularVelocityRadPerSec = DegreesPerSecond.of(240);
  public static final AngularAcceleration maxAngularAcceleratRadPerSecSq =
      DegreesPerSecondPerSecond.of(480);

  public static final Angle angularTolerance = Degrees.of(5);

  public PathConstraints getPathConstraints() {
    return new PathConstraints(
        MetersPerSecond.of(translationMaxVelMpsTN.get()),
        MetersPerSecondPerSecond.of(translationMaxAccelMpsSqTN.get()),
        RadiansPerSecond.of(angularMaxVelRadPerSec()),
        RadiansPerSecondPerSecond.of(angularMaxAccelRadPerSecSq()));
  }

  // -----------------------------
  // TRANSLATION PID (raw doubles)
  // -----------------------------
  /** translationKP Tunable Number */
  public static final TunableNumber translationKPTN =
      new TunableNumber("auto/translation_kP", translation_kP);

  /** translationKI Tunable Number */
  public static final TunableNumber translationKITN =
      new TunableNumber("auto/translation_kI", 0.05);

  /** translationKD Tunable Number */
  public static final TunableNumber translationKDTN =
      new TunableNumber("auto/translation_kD", translation_kD);

  // -----------------------------
  // ANGULAR PID (raw doubles)
  // -----------------------------
  /** angularKP Tunable Number */
  public static final TunableNumber angularKPTN = new TunableNumber("auto/angular_kP", angular_kP);

  /** angularKI Tunable Number */
  public static final TunableNumber angularKITN = new TunableNumber("auto/angular_kI", angular_kI);

  /** angularKIzone Tunable Number */
  public static final TunableNumber angularKIzoneTN =
      new TunableNumber("auto/angular_kIzone", angular_kIzone);

  /** angularKD Tunable Number */
  public static final TunableNumber angularKDTN = new TunableNumber("auto/angular_kD", angular_kD);

  // -----------------------------
  // TRANSLATION CONSTRAINTS
  // (stored as doubles, converted to units)
  // -----------------------------
  /** translationMaxVelMps Tunable Number */
  public static final TunableNumber translationMaxVelMpsTN =
      new TunableNumber(
          "auto/translationMaxVel_mps", translationMaxVelocityMps.in(MetersPerSecond));

  /** translationMaxAccelMpsSq Tunable Number */
  public static final TunableNumber translationMaxAccelMpsSqTN =
      new TunableNumber(
          "auto/translationMaxAccel_mps2",
          translationMaxAcceleratMpsSq.in(MetersPerSecondPerSecond));

  public static double translationMaxVel() {
    return translationMaxVelMpsTN.get();
  }

  public static double translationMaxAccel() {
    return translationMaxAccelMpsSqTN.get();
  }

  // -----------------------------
  // ANGULAR CONSTRAINTS (DEGREES)
  // -----------------------------
  /** angularMaxVelDeg Tunable Number */
  public static final TunableNumber angularMaxVelDegTN =
      new TunableNumber(
          "auto/angularMaxVel_degPerSec", maxAngularVelocityRadPerSec.in(DegreesPerSecond));

  /** angularMaxAccelDeg Tunable Number */
  public static final TunableNumber angularMaxAccelDegTN =
      new TunableNumber(
          "auto/angularMaxAccel_degPerSec2",
          maxAngularAcceleratRadPerSecSq.in(DegreesPerSecondPerSecond));

  /** angularToleranceDeg Tunable Number */
  public static final TunableNumber angularToleranceDegTN =
      new TunableNumber("auto/angularTolerance_deg", angularTolerance.in(Degrees));

  // Convert to radians for controllers
  public static double angularMaxVelRadPerSec() {
    return Math.toRadians(angularMaxVelDegTN.get());
  }

  public static double angularMaxAccelRadPerSecSq() {
    return Math.toRadians(angularMaxAccelDegTN.get());
  }

  public static double angularToleranceRad() {
    return Math.toRadians(angularToleranceDegTN.get());
  }
}
