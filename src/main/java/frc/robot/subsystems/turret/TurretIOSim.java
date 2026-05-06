package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

public class TurretIOSim implements TurretIO {
    private LinearSystemSim<N2, N1, N2> sim;
    private double appliedVolts = 0.0;
    private double simkV = 0.05;
    private double simkA = 0.05;
    public TurretIOSim() {
        sim = new LinearSystemSim<>(
            LinearSystemId.identifyPositionSystem(simkV, simkA)
        );
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
        System.out.println("target goal deg" + angle.getDegrees());
        sim.setInput(angle.getRadians());
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        //TODO: Simulate encoder?
        inputs.builtinEncoderAngle = new Rotation2d(Degree.of(5));
        inputs.appliedVoltage = Volts.of(appliedVolts);
    }
}
