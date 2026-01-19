package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.util.AlienceColorCoordinateFlip;
import frc.lib.util.TunableNumber;

public final class AutoConstants {
  // FIXME: The below constants are used in the example auto, and must be
  // tuned to specific robot
  public static final double kMaxSpeedMetersPerSecond = 10;
  public static final double kMaxAccelerationMetersPerSecondSquared = 3;
  public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
  public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

  public static final double translation_kP = 2.25;
  public static final double translation_kI = 0.05;
  public static final double translation_kD = 0;
  public static final double rotation_kP = 0.45;
  public static final double rotation_kI = 0;
  public static final double rotation_kD = 0.05;
  public static final double rotationMaxAccel = 120;
  public static final double rotationMaxVel = 240;

  public static final double autoMaxVelocityMps = 5;
  public static final double autoMaxAcceleratMpsSq = 15;
  public static final double maxAngularVelocityRps = Rotation2d.fromDegrees(240).getRadians();
  public static final double maxAngularAcceleratRpsSq = Rotation2d.fromDegrees(480).getRadians();

  public final TunableNumber autoMaxVelocityTunableNumber =
      new TunableNumber("autoMaxVelocity", autoMaxVelocityMps);
  public final TunableNumber autoMaxAcceleratMpsSqTunableNumber =
      new TunableNumber("autoMaxAcceleratMpsSq", autoMaxAcceleratMpsSq);
  public final TunableNumber maxAngularVelocityRpstunautoMaxAcceleratMpsSqTunableNumber =
      new TunableNumber("maxAngularVelocityRps", maxAngularVelocityRps);
  public final TunableNumber maxAngularAcceleratRpsSqtunautoMaxAcceleratMpsSqTunableNumber =
      new TunableNumber("maxAngularAcceleratRpsSq", maxAngularAcceleratRpsSq);

  public static final double kPXController = 1.5;
  public static final double kPYController = 1.5;
  public static final double kPThetaController = 3;

  public static final PathConstraints pathConstraints =
      new PathConstraints(
          autoMaxVelocityMps,
          kMaxAccelerationMetersPerSecondSquared,
          maxAngularVelocityRps,
          maxAngularAcceleratRpsSq);

  /* Constraint for the motion profilied robot angle controller */
  public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
      new TrapezoidProfile.Constraints(
          kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
}
