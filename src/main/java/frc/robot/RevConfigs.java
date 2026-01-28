package frc.robot;

import frc.robot.config.RobotConfig;

public class RevConfigs {

  // BOTH OF THESE CONVERT FROM ROTATIONS TO ROTATIONS

  public static double CANCoderAngleToNeoEncoder(double CANCoderAngle) {
    // return CANCoderAngle / (360) * Constants.Swerve.angleGearRatio * 42; //BAD
    // return CANCoderAngle / (2 * Math.PI) * Constants.Swerve.angleGearRatio * 42; //BAD
    return (CANCoderAngle * RobotConfig.SWERVECONFIG.angleGearRatio());
  }

  public static double NeoEncoderAngleToCANCoder(double NeoEncoderAngle) {
    // return NeoEncoderAngle * (360) / Constants.Swerve.angleGearRatio / 42; //BAD
    // return NeoEncoderAngle * (2 * Math.PI) / Constants.Swerve.angleGearRatio / 42; //BAD
    return NeoEncoderAngle / RobotConfig.SWERVECONFIG.angleGearRatio(); // THIS ONE ACTUALLY WORKS
  }
}
