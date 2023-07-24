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

  //TODO: Nothing will be showing up on Shuffleboard as no one creates the ShuffleboardInfo class and adds the tabs. 
  // Each tab needs to be added in the constructor of this class. See last year's code as an example

  public RobotContainer() {
    swerveDrive = new SwerveDrive();
    driverController = new CommandXboxController(DriverConstants.DRIVER_CONTROLLER_PORT);
    configureBindings();

    //TODO: Shouldn't the default command be AbsoluteDriveState to allow the driver to control which way the robot faces rather than rate of turn? 
    //TODO: Do you need to negate the LeftY so that pointing forwards goes away from driver station?
    swerveDrive.setDefaultCommand(new FieldCentricDriveState(swerveDrive,
     driverController::getLeftX,
      driverController::getLeftY,
      driverController::getRightY));
  }

  private void configureBindings() {
    driverController.a().onTrue(new LockWheelsState(swerveDrive));
    //TODO: Can there be a button that when you hold it that it just makes the robot spin as it drives? For example, to spin off a robot playing defence?
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  // TODO: Typo.. isBreaked should be isBraked
  public void setDriveBaseBrake(boolean isBreaked){
    swerveDrive.setMotorBrake(isBreaked);
  }
}
