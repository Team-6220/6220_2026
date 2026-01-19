// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

/** Add your docs here. */
public class GyroIONavX implements GyroIO{
        private final AHRS navx = new AHRS(NavXComType.kMXP_SPI, NavXUpdateRate.k200Hz);
    
        @Override
        public void updateInputs(GyroIOInputs inputs) {
            inputs.yawPosition = navx.getRotation2d();
        }
    }