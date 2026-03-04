// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

/** Simulation implementation of {@link ShooterIO}. */
public class ShooterIOSim implements ShooterIO {

  private double motor41VelocityRPS = 0.0;
  private double motor1VelocityRPS = 0.0;
  private double motor9VelocityRPS = 0.0;
  private double motor31VelocityRPS = 0.0;
  private double motor2VelocityRPS = 0.0;

  public ShooterIOSim() {}

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.motor41VelocityRPS = motor41VelocityRPS;
    inputs.motor1VelocityRPS = motor1VelocityRPS;
    inputs.motor9VelocityRPS = motor9VelocityRPS;
    inputs.motor31VelocityRPS = motor31VelocityRPS;
    inputs.motor2VelocityRPS = motor2VelocityRPS;

    // Voltages and currents are not simulated; leave as defaults (0.0)
  }

  @Override
  public void setVelocityRPS(double rps) {
    motor41VelocityRPS = rps;
    motor1VelocityRPS = rps;
    motor9VelocityRPS = rps;
    motor31VelocityRPS = rps;
    motor2VelocityRPS = rps;
  }

  @Override
  public void stop() {
    motor41VelocityRPS = 0.0;
    motor1VelocityRPS = 0.0;
    motor9VelocityRPS = 0.0;
    motor31VelocityRPS = 0.0;
    motor2VelocityRPS = 0.0;
  }

  @Override
  public void stopTopGroup() {
    motor41VelocityRPS = 0.0;
    motor1VelocityRPS = 0.0;
  }

  @Override
  public void stopBottomGroup() {
    motor9VelocityRPS = 0.0;
    motor2VelocityRPS = 0.0;
    motor31VelocityRPS = 0.0;
  }

  @Override
  public void setPercentOutput(double percent) {
    // Simulate velocity as a fraction of a nominal max RPS
    double simRPS = percent * 100.0;
    motor41VelocityRPS = simRPS;
    motor1VelocityRPS = simRPS;
    motor9VelocityRPS = simRPS;
    motor31VelocityRPS = simRPS;
    motor2VelocityRPS = simRPS;
  }

  @Override
  public void setTopGroupPercent(double percent) {
    double simRPS = percent * 100.0;
    motor41VelocityRPS = simRPS;
    motor1VelocityRPS = simRPS;
  }

  @Override
  public void setBottomGroupPercent(double percent) {
    double simRPS = percent * 100.0;
    motor9VelocityRPS = simRPS;
    motor2VelocityRPS = simRPS;
    motor31VelocityRPS = simRPS;
  }
}
