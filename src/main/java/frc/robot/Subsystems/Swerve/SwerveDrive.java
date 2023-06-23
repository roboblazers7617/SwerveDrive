// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;


public class SwerveDrive extends SubsystemBase {
  /** Creates a new SwerveDrive. */
  private swervelib.SwerveDrive drivetrain;
  public SwerveDrive() {
    try{
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
      drivetrain = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive();
    }
    catch(Exception e){
      throw new RuntimeException(e);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
