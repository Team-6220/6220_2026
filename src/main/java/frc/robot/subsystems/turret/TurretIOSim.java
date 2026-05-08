package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurretIOSim implements TurretIO {
  private LinearSystemSim<N2, N1, N2> sim;
  private double appliedVolts = 0.0;
  private double simkV = 0.05;
  private double simkA = 0.05;
  // Simple integrator state for sim (angle in radians, velocity in rad/s)
  private double simAngle = 0.0;
  private double simVel = 0.0;
  // Profiled PID controller for simulated position control
  private final ProfiledPIDController profiledPID;
  private double goalAngle = 0.0;

  public TurretIOSim() {
    sim = new LinearSystemSim<>(LinearSystemId.identifyPositionSystem(simkV, simkA));
    profiledPID =
        new ProfiledPIDController(
            TurretConstants.kP,
            TurretConstants.kI,
            TurretConstants.kD,
            new TrapezoidProfile.Constraints(
                TurretConstants.MAX_VEL_RAD_PER_SEC, TurretConstants.MAX_ACCEL_RAD_PER_SEC_S));
  }

  @Override
  public boolean atSetpoint() {
    return false; // subsystem handles this
  }

  @Override
  public void setVoltageOut(Voltage voltsOut) {
    appliedVolts = voltsOut.in(Volts);
  }

  @Override
  public void driveToGoal(Rotation2d angle) {
    // store goal for profiled PID
    goalAngle = angle.getRadians();
    SmartDashboard.putNumber("turret goal degrees", angle.getDegrees());
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Very small, simple simulation: integrate applied volts -> angular acceleration -> velocity ->
    // angle
    // This is intentionally simple and can be replaced with a more detailed plant.
    double dt = 0.02; // assume 20ms periodic

    // PID output (units treated as volts for simulation convenience)
    double pidOut = profiledPID.calculate(simAngle, goalAngle);

    // Feedforward from TurretConstants (expects velocity units compatible with its tuning)
    double ff = TurretConstants.TURRET_FEEDFORWARD.calculate(simVel);

    appliedVolts = Math.max(-12.0, Math.min(12.0, pidOut + ff));

    // Simple physics: accelerate proportionally to applied volts, damped by velocity
    double acceleration = appliedVolts * simkA - 0.5 * simVel;
    simVel += acceleration * dt;
    simAngle += simVel * dt;

    inputs.builtinEncoderAngle = new Rotation2d(simAngle);
    inputs.appliedVoltage = Volts.of(appliedVolts);
    inputs.motorCurrent = Amps.of(Math.abs(appliedVolts) * 2.0);
    inputs.motorTemperature = Celsius.of(30.0 + Math.abs(appliedVolts) * 2.0);
  }

  @Override
  public void resetBuiltInEncoderAngle(double angleRad, double angularVelocity) {
    // simple no-op for sim: set internal sim input to match the angle
    sim.setInput(angleRad);
  }

  @Override
  public void resetOutsideInEncoderAngle(double angleRad, double angularVelocity) {
    // sim has only one encoder; no-op but present for completeness
    sim.setInput(angleRad);
  }
}
