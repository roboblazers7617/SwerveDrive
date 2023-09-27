// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shuffleboard;

import frc.robot.Subsystems.Swerve.SwerveDrive;

/** Add your docs here. */
public class DriverStationTab extends ShuffleboardTabBase {
    private final SwerveDrive swerveDrive;
    public DriverStationTab(SwerveDrive swerveDrive){
        this.swerveDrive = swerveDrive;
    }
    
    @Override
    public void update() {
    }
}
