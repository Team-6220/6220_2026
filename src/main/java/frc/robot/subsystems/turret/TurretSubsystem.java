package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;

/** Minimal turret subsystem with a PID controller for angle. Meant as a compile-ready stub. */
public class TurretSubsystem extends SubsystemBase{
  private final TurretIO io;
  private final TurretIOInputs inputs = new TurretIOInputs();

  public TurretSubsystem(TurretIO io){
    this.io = io;
  }

  public void driveToGoal(Rotation2d angle) {
    io.driveToGoal(angle);
  }

  public boolean atSetpoint() {
    return io.atSetpoint();
  }

  public void setVoltageOut(Voltage voltsOut){
    io.setVoltageOut(voltsOut);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }
}
