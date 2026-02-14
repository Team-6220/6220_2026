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

  public static final double translation_kP = 7;
  public static final double translation_kI = 0.05;
  public static final double translation_kD = 0;
  public static final double angular_kP = 6;
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

  public final TunableNumber autoMaxVelocityTN =
      new TunableNumber("autoMaxVelocityMps", translationMaxVelocityMps.in(MetersPerSecond));

  public final TunableNumber autoMaxAccelTN =
      new TunableNumber(
          "autoMaxAccelMpsSq", translationMaxAcceleratMpsSq.in(MetersPerSecondPerSecond));

  public final TunableNumber autoMaxAngularVelTN =
      new TunableNumber("autoMaxAngularVelRps", maxAngularVelocityRadPerSec.in(RadiansPerSecond));

  public final TunableNumber autoMaxAngularAccelTN =
      new TunableNumber(
          "autoMaxAngularAccelRpsSq", maxAngularAcceleratRadPerSecSq.in(RadiansPerSecondPerSecond));

  public PathConstraints getPathConstraints() {
    return new PathConstraints(
        MetersPerSecond.of(autoMaxVelocityTN.get()),
        MetersPerSecondPerSecond.of(autoMaxAccelTN.get()),
        RadiansPerSecond.of(autoMaxAngularVelTN.get()),
        RadiansPerSecondPerSecond.of(autoMaxAngularAccelTN.get()));
  }
}
