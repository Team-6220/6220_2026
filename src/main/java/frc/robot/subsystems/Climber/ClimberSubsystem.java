// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    
    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }
    
    /**
     * Drive the left climber motor
     * @param speed Motor speed from -1.0 to 1.0 (automatically clamped to ±0.5)
     */
    public void simpleDriveLeft(double speed) {
        io.setLeftMotor(speed);
    }
    
    // TODO: Add right motor control when installed on full robot
    // /**
    //  * Drive the right climber motor
    //  * @param speed Motor speed from -1.0 to 1.0 (automatically clamped to ±0.5)
    //  */
    // public void simpleDriveRight(double speed) {
    //     io.setRightMotor(speed);
    // }
    
    /**
     * Set the climber servo position
     * @param position Servo position from 0.0 to 1.0
     */
    public void setServoPosition(double position) {
        io.setServoPosition(position);
    }
    
    /**
     * Get the current servo position
     * @return Current position from 0.0 to 1.0
     */
    public double getServoPosition() {
        return inputs.servoPosition;
    }
    
    /**
     * Get the left climber height based on encoder position
     * @return Height in configured units (inches/meters based on coefficient)
     */
    public double getLeftClimberHeight() {
        return inputs.leftClimberHeight;
    }
    
    // TODO: Add right climber height when installed on full robot
    // /**
    //  * Get the right climber height based on encoder position
    //  * @return Height in configured units (inches/meters based on coefficient)
    //  */
    // public double getRightClimberHeight() {
    //     return inputs.rightClimberHeight;
    // }
    
    /**
     * Stop the climber motor(s)
     */
    public void stop() {
        simpleDriveLeft(0);
        // TODO: Add right motor stop when installed on full robot
        // simpleDriveRight(0);
    }
}