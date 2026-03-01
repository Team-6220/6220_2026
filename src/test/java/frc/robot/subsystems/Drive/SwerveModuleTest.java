package frc.robot.subsystems.Drive;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.RevConfigs;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SwerveModuleTest {

  /** Fake IO that lets tests (1) inject sensor readings and (2) capture outputs. */
  private static class FakeIO implements SwerveModuleIO {
    // Inputs we want updateInputs() to report
    public double drivePosMeters = 0.0;
    public double driveVelMps = 0.0;
    public double anglePosRad = 0.0;
    public Rotation2d absAngle = new Rotation2d();

    // Captured outputs
    public DutyCycleOut lastDutyCycle = null;
    public VelocityVoltage lastVelocityVoltage = null;
    public Voltage lastDriveVolts = null;
    public Double lastAngleSetpoint = null;
    public Rotation2d lastResetOffset = null;

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
      inputs.drivePositionMeters = Units.Meter.of(drivePosMeters);
      inputs.driveVelocityMps = Units.MetersPerSecond.of(driveVelMps);
      inputs.anglePositionRad = Units.Radian.of(anglePosRad);
      inputs.absoluteAngle = absAngle;
    }

    @Override
    public void setDriveControlDutyCycle(DutyCycleOut driveControl) {
      // copy so later mutations don't affect assertions
      lastDutyCycle = new DutyCycleOut(driveControl.Output);
    }

    @Override
    public void setDriveControlVelocity(VelocityVoltage driveControl) {
      lastVelocityVoltage = new VelocityVoltage(driveControl.Velocity);
      lastVelocityVoltage.FeedForward = driveControl.FeedForward;
    }

    @Override
    public void setDriveVoltage(Voltage volts) {
      lastDriveVolts = volts;
    }

    @Override
    public void setAnglePosition(double setpoint) {
      lastAngleSetpoint = setpoint;
    }

    @Override
    public void resetToAbsolute(Rotation2d offset) {
      lastResetOffset = offset;
    }
  }

  private FakeIO io;
  private SwerveModule module;

  @BeforeEach
  void setup() {
    io = new FakeIO();
    var config = new SwerveModuleConstants(1, 2, 3, Rotation2d.fromDegrees(12.34));
    module = new SwerveModule(0, config, io);
  }

  @Test
  void getStateReflectsInputs() {
    io.driveVelMps = 2.5;
    io.anglePosRad = Math.PI / 2.0; // 90 deg
    module.periodic();

    var state = module.getState();
    assertEquals(2.5, state.speedMetersPerSecond, 1e-9);
    assertEquals(Math.PI / 2.0, state.angle.getRadians(), 1e-9);
  }

  @Test
  void openLoopDriveUsesDutyCycleScaledByMaxSpeed() {
    io.anglePosRad = 0.0;
    module.periodic();

    var desired = new SwerveModuleState(1.0, Rotation2d.fromDegrees(0));
    module.setDesiredState(desired, true);

    assertNotNull(io.lastDutyCycle, "Expected open-loop to set DutyCycleOut");
    double expected = 1.0 / SwerveConstants.maxSpeed();
    assertEquals(expected, io.lastDutyCycle.Output, 1e-9);
  }

  @Test
  void closedLoopDriveUsesVelocityRpsAndFeedforward() {
    io.anglePosRad = 0.0;
    module.periodic();

    double speedMps = 2.0;
    var desired = new SwerveModuleState(speedMps, Rotation2d.fromDegrees(0));
    module.setDesiredState(desired, false);

    assertNotNull(io.lastVelocityVoltage, "Expected closed-loop to set VelocityVoltage");
    double expectedRps = Conversions.MPSToRPS(speedMps, SwerveConstants.wheelCircumference());
    assertEquals(expectedRps, io.lastVelocityVoltage.Velocity, 1e-9);

    // Feedforward should be nonzero for positive speed (given nonzero kS/kV in constants)
    assertTrue(
        io.lastVelocityVoltage.FeedForward > 0.0,
        "Expected positive feedforward for positive speed");
  }

  @Test
  void optimizeFlipsSpeedWhenTargetIsOppositeDirection() {
    // Current angle ~0, desired angle 180 deg should optimize to angle ~0 and speed negative
    io.anglePosRad = 0.0;
    module.periodic();

    var desired = new SwerveModuleState(1.0, Rotation2d.fromDegrees(180));
    module.setDesiredState(desired, true);

    // After optimize(), setAnglePosition should be close to "0 rotations" (converted to NEO units)
    assertNotNull(io.lastAngleSetpoint);
    double expectedNeo = RevConfigs.CANCoderAngleToNeoEncoder(0.0);
    assertEquals(expectedNeo, io.lastAngleSetpoint, 1e-9);

    // And duty cycle should reflect the flipped speed (-1/maxSpeed)
    assertNotNull(io.lastDutyCycle);
    double expectedDuty = -1.0 / SwerveConstants.maxSpeed();
    assertEquals(expectedDuty, io.lastDutyCycle.Output, 1e-9);
  }

  @Test
  void resetToAbsolutePassesConfigOffsetToIO() {
    module.resetToAbsolute();
    assertNotNull(io.lastResetOffset);
    assertEquals(12.34, io.lastResetOffset.getDegrees(), 1e-9);
  }

  /**
   * This test is meant to catch the "angle wrap" issue.
   *
   * If your angle pipeline expects [-0.5, +0.5) rotations, then any commanded angle should be wrapped
   * into [-0.5, +0.5) BEFORE converting to the NEO setpoint.
   *
   * If this fails, it's a sign you should wrap with inputModulus(rot, -0.5, +0.5).
   */
  @Test
  void desiredAngleShouldBeWrappedToMinus0p5ToPlus0p5RotationsBeforeSetpoint() {
    io.anglePosRad = 0.0;
    module.periodic();

    // 270 degrees => -0.25 rotations if represented as -180..180
    var desired = new SwerveModuleState(0.0, Rotation2d.fromDegrees(270));
    module.setDesiredState(desired, true);

    assertNotNull(io.lastAngleSetpoint);

    double rawRot = desired.angle.getRotations();
    double expectedWrappedNeo = RevConfigs.CANCoderAngleToNeoEncoder(rawRot);

    assertEquals(
        expectedWrappedNeo,
        io.lastAngleSetpoint,
        1e-9,
        "Angle setpoint should be wrapped into [0,1) rotations before conversion");
  }
}
