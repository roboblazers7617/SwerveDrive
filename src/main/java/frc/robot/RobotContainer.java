// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriverConstants;
import frc.robot.Subsystems.Swerve.SwerveDrive;
import frc.robot.Subsystems.Swerve.States.FieldCentricDriveState;
import frc.robot.Subsystems.Swerve.States.LockWheelsState;

public class RobotContainer {
  private SwerveDrive swerveDrive;
  private CommandXboxController driverController;
  public RobotContainer() {
    swerveDrive = new SwerveDrive();
    driverController = new CommandXboxController(DriverConstants.DRIVER_CONTROLLER_PORT);
    configureBindings();

    swerveDrive.setDefaultCommand(new FieldCentricDriveState(swerveDrive,
     driverController::getLeftX,
      driverController::getLeftY,
      driverController::getRightY));
  }

  private void configureBindings() {
    driverController.a().onTrue(new LockWheelsState(swerveDrive));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void setDriveBaseBrake(boolean isBreaked){
    swerveDrive.setMotorBrake(isBreaked);
  }
}
