package frc.robot.subsystems.turret;

/**
 * Lightweight physics simulation for a turret assembly.
 *
 * <p>Model details (configurable): - Simple DC motor with resistance R (ohms), torque constant Kt
 * (Nm/A), and velocity constant Kv (rad/s per V) - Gearbox with a gear ratio (motor rotations per
 * turret rotation). Torque is multiplied by gearRatio. - Inertia (moment of inertia) of the turret
 * (kg*m^2) - Viscous friction (N*m / (rad/s)) and Coulomb friction (static torque threshold)
 *
 * <p>This class is deterministic and not tied to WPILib simulation classes so it can be used in
 * unit tests or simple offline simulations. Angles are in radians, angular velocities in rad/s,
 * voltages in volts.
 */
public final class TurretPhysicsSim {
  // Motor/electrical
  private final double motorResistance; // ohms
  private final double motorKt; // Nm / A
  private final double motorKv; // rad/s per V (back-EMF constant)

  // Mechanical
  private final double gearRatio; // motor revs per turret rev (e.g., 10 means motor turns 10x)
  private final double inertia; // kg*m^2 (turret reflected to turret side)
  private final double viscousFriction; // N*m per (rad/s)
  private final double coulombFriction; // N*m

  // State
  private double angleRad = 0.0; // turret angle (rad)
  private double angularVelocity = 0.0; // rad/s (turret side)
  private double appliedVoltage = 0.0; // volts (commanded to motor)

  // Derived convenience
  private final double maxVoltage;

  /**
   * Create a turret physics sim with explicit parameters.
   *
   * <p>All parameters must be in SI units (radians, seconds, meters, kilograms, volts, amperes).
   */
  public TurretPhysicsSim(
      double motorResistance,
      double motorKt,
      double motorKv,
      double gearRatio,
      double inertia,
      double viscousFriction,
      double coulombFriction,
      double maxVoltage) {
    this.motorResistance = motorResistance;
    this.motorKt = motorKt;
    this.motorKv = motorKv;
    this.gearRatio = gearRatio;
    this.inertia = inertia;
    this.viscousFriction = viscousFriction;
    this.coulombFriction = coulombFriction;
    this.maxVoltage = Math.abs(maxVoltage);
  }

  /**
   * Construct a sim with reasonable default parameters for a small FRC turret. Defaults chosen to
   * be conservative placeholders — tune for your robot.
   */
  public TurretPhysicsSim() {
    // Defaults (placeholder values):
    // - motorResistance (ohms): 0.1
    // - motorKt (Nm/A): 0.018 (small brushless),
    // - motorKv (rad/s per V): 4000 RPM/V -> rad/s per V = 4000 * 2*pi / 60 ~ 418.88
    //   but this is highly motor-specific. We choose a lower Kv so numbers are reasonable.
    double defaultMotorResistance = 0.1;
    double defaultMotorKt = 0.02;
    double defaultMotorKv = 400.0; // rad/s per V
    double defaultGearRatio = 10.0; // motor revs per turret rev
    double defaultInertia = 0.02; // kg*m^2
    double defaultViscousFriction = 0.01; // N*m per (rad/s)
    double defaultCoulombFriction = 0.05; // N*m
    double defaultMaxVoltage = 12.0;
    this.motorResistance = defaultMotorResistance;
    this.motorKt = defaultMotorKt;
    this.motorKv = defaultMotorKv;
    this.gearRatio = defaultGearRatio;
    this.inertia = defaultInertia;
    this.viscousFriction = defaultViscousFriction;
    this.coulombFriction = defaultCoulombFriction;
    this.maxVoltage = defaultMaxVoltage;
  }

  /** Set the commanded voltage applied to the motor (clamped to +/- maxVoltage). */
  public void setVoltage(double volts) {
    if (volts > maxVoltage) volts = maxVoltage;
    if (volts < -maxVoltage) volts = -maxVoltage;
    this.appliedVoltage = volts;
  }

  /** Advance the simulation by dt seconds. */
  public void update(double dtSeconds) {
    if (dtSeconds <= 0) return;

    // Convert turret-side angular velocity to motor-side
    double omegaMotor = angularVelocity * gearRatio; // rad/s at motor shaft

    // Back-EMF voltage from motor (V)
    double backEmf = omegaMotor / motorKv;

    // Motor current (A)
    double motorCurrent = (appliedVoltage - backEmf) / motorResistance;

    // Motor torque (Nm at motor)
    double motorTorque = motorKt * motorCurrent;

    // Torque reflected to turret side (multiply motor torque by gear ratio)
    double torqueAtTurret = motorTorque * gearRatio;

    // Friction torques
    double viscous = viscousFriction * angularVelocity;
    double coulomb =
        coulombFriction
            * Math.signum(angularVelocity != 0 ? angularVelocity : (appliedVoltage >= 0 ? 1 : -1));

    // Net torque on turret (Nm)
    double netTorque = torqueAtTurret - viscous - coulomb;

    // Angular acceleration (rad/s^2)
    double alpha = netTorque / inertia;

    // Integrate (semi-implicit Euler)
    angularVelocity += alpha * dtSeconds;
    angleRad += angularVelocity * dtSeconds;
  }

  /** Reset state to given angle and velocity. */
  public void reset(double angleRad, double angularVelocity) {
    this.angleRad = angleRad;
    this.angularVelocity = angularVelocity;
    this.appliedVoltage = 0.0;
  }

  public double getAngleRad() {
    return angleRad;
  }

  public double getAngularVelocity() {
    return angularVelocity;
  }

  /** Approximate motor current in amps for the last applied voltage. */
  public double getMotorCurrent() {
    double omegaMotor = angularVelocity * gearRatio;
    double backEmf = omegaMotor / motorKv;
    return (appliedVoltage - backEmf) / motorResistance;
  }

  /** Approximate torque at the turret (Nm). */
  public double getTorqueAtTurret() {
    return motorKt * getMotorCurrent() * gearRatio;
  }

  public double getAppliedVoltage() {
    return appliedVoltage;
  }
}
