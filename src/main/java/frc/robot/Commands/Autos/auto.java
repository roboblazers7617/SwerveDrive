// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Auton;
import frc.robot.Subsystems.Swerve.SwerveDrive;

/** Add your docs here. */
public class auto {
  
  
  
  
  
  public static Command testPath(SwerveDrive swerveDrive) {
    SwerveDrive drive = new SwerveDrive(); // Create an instance of SwerveDrive
    Command path = drive.createPathPlannerCommand("path", Auton.constraint, Auton.eventMap, Auton.yAutoPID, Auton.angleAutoPID, false);
    return path;
}
}