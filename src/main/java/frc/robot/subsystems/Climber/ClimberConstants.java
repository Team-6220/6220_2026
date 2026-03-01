package frc.robot.subsystems.Climber;

public class ClimberConstants {
  // Motor CAN IDs
  public static final int climberDriverLeftID = 20;
  public static final int climberDriverRightID = 13;

  // Motor inversions
  public static final boolean leftMotorInverted = false;
  public static final boolean rightMotorInverted = true;
  // Servo configuration
  public static final int leftServoPWMPort = 1;
  public static final int rightServoPWMPort = 0;

  // Servo position limits
  // Position when servo is deployed (button pressed)
  public static final double SERVO_MAX = 0.75;
  // Position when servo is retracted (button unpressed)
  public static final double SERVO_MIN = 0.25;

  // Stall current threshold for zero-height detection (bottom limit)
  // When motor current exceeds this value, the climber has hit bottom
  public static final double STALL_CURRENT_THRESHOLD_BOTTOM =
      6.0; // Amperes / NOTE: THIS WILL ONLY BE USED WHEN TUNING MODE in Constants.java is FALSE

  // Stall current threshold for max-height detection (top limit)
  // When motor current exceeds this value, the climber has hit top
  public static final double STALL_CURRENT_THRESHOLD_TOP =
      90.0; // Amperes / NOTE: THIS WILL ONLY BE USED WHEN TUNING MODE in Constants.java is FALSE

  // Velocity threshold for stall detection (only used for bottom)
  // Motor must be running at this velocity to distinguish stall from acceleration
  public static final double VELOCITY_THRESHOLD = 4000.0;

  // Motor speed configuration
  // Speed for climber movement (0.0 to 1.0)
  public static final double CLIMBER_SPEED = 0.75;
}
