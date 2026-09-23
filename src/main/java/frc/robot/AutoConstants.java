package frc.robot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.DegreesPerSecondPerSecond;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;

// TODO: AUTO - PathPlanner doesn't support WPILib 2027 alpha 7 yet.
// import com.pathplanner.lib.path.PathConstraints;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearAcceleration;
import org.wpilib.units.measure.LinearVelocity;
import frc.lib.util.TunableHelper;
import org.wpilib.tunable.TunableDouble;

public final class AutoConstants {
  // FIXME: The below constants are used in the example auto, and must be
  // tuned to specific robot

  public static final double translation_kP =
      2.25; // Used to be 7 but that was never used in autobuilder(and maybe way too high)
  public static final double translation_kI = 0.05;
  public static final double translation_kD = 0;
  public static final double angular_kP =
      3.5; // Used to be 6 but that was never used in autobuilder(and maybe way too high)
  public static final double angular_kI = 0.1;
  public static final double angular_kIzone = 0.05;
  public static final double angular_kD = 0.02;

  public static final LinearVelocity translationMaxVelocityMps = MetersPerSecond.of(5);
  public static final LinearAcceleration translationMaxAcceleratMpsSq =
      MetersPerSecondPerSecond.of(15);
  public static final AngularVelocity maxAngularVelocityRadPerSec = DegreesPerSecond.of(240);
  public static final AngularAcceleration maxAngularAcceleratRadPerSecSq =
      DegreesPerSecondPerSecond.of(480);

  public static final Angle angularTolerance = Degrees.of(5);

  // TODO: AUTO - PathPlanner path constraints, disabled until PathPlanner supports WPILib 2027
  // alpha 7. The tunable limits below are still available to any other path follower.
  // public static PathConstraints getPathConstraints() {
  //   return new PathConstraints(
  //       MetersPerSecond.of(translationMaxVelMpsTN.get()),
  //       MetersPerSecondPerSecond.of(translationMaxAccelMpsSqTN.get()),
  //       RadiansPerSecond.of(angularMaxVelRadPerSec()),
  //       RadiansPerSecondPerSecond.of(angularMaxAccelRadPerSecSq()));
  // }

  // -----------------------------
  // TRANSLATION PID (raw doubles)
  // -----------------------------
  /** translationKP Tunable Number */
  public static final TunableDouble translationKPTN =
      TunableHelper.addDouble("auto/translation_kP", translation_kP);

  /** translationKI Tunable Number */
  public static final TunableDouble translationKITN =
      TunableHelper.addDouble("auto/translation_kI", translation_kI);

  /** translationKD Tunable Number */
  public static final TunableDouble translationKDTN =
      TunableHelper.addDouble("auto/translation_kD", translation_kD);

  // -----------------------------
  // ANGULAR PID (raw doubles)
  // -----------------------------
  /** angularKP Tunable Number */
  public static final TunableDouble angularKPTN = TunableHelper.addDouble("auto/angular_kP", angular_kP);

  /** angularKI Tunable Number */
  public static final TunableDouble angularKITN = TunableHelper.addDouble("auto/angular_kI", angular_kI);

  /** angularKIzone Tunable Number */
  public static final TunableDouble angularKIzoneTN =
      TunableHelper.addDouble("auto/angular_kIzone", angular_kIzone);

  /** angularKD Tunable Number */
  public static final TunableDouble angularKDTN = TunableHelper.addDouble("auto/angular_kD", angular_kD);

  // -----------------------------
  // TRANSLATION CONSTRAINTS
  // (stored as doubles, converted to units)
  // -----------------------------
  /** translationMaxVelMps Tunable Number */
  public static final TunableDouble translationMaxVelMpsTN =
      TunableHelper.addDouble(
          "auto/translationMaxVel_mps", translationMaxVelocityMps.in(MetersPerSecond));

  /** translationMaxAccelMpsSq Tunable Number */
  public static final TunableDouble translationMaxAccelMpsSqTN =
      TunableHelper.addDouble(
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
  public static final TunableDouble angularMaxVelDegTN =
      TunableHelper.addDouble(
          "auto/angularMaxVel_degPerSec", maxAngularVelocityRadPerSec.in(DegreesPerSecond));

  /** angularMaxAccelDeg Tunable Number */
  public static final TunableDouble angularMaxAccelDegTN =
      TunableHelper.addDouble(
          "auto/angularMaxAccel_degPerSec2",
          maxAngularAcceleratRadPerSecSq.in(DegreesPerSecondPerSecond));

  /** angularToleranceDeg Tunable Number */
  public static final TunableDouble angularToleranceDegTN =
      TunableHelper.addDouble("auto/angularTolerance_deg", angularTolerance.in(Degrees));

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
