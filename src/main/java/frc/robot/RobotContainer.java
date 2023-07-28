// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriverConstants;
import frc.robot.Shuffleboard.ShuffleboardInfo;
import frc.robot.Shuffleboard.ShuffleboardTabBase;
import frc.robot.Shuffleboard.SwerveTab;
import frc.robot.Subsystems.Swerve.SwerveDrive;
import frc.robot.Subsystems.Swerve.States.AbsoluteDriveState;
import frc.robot.Subsystems.Swerve.States.FieldCentricDriveState;
import frc.robot.Subsystems.Swerve.States.LockWheelsState;
import frc.team4272.globals.MathUtils;

public class RobotContainer {
  private SwerveDrive swerveDrive;
  private CommandXboxController driverController;
  public String path;

  public RobotContainer() {
    swerveDrive = new SwerveDrive();
    driverController = new CommandXboxController(DriverConstants.DRIVER_CONTROLLER_PORT);
    configureBindings();

    ArrayList<ShuffleboardTabBase> tabs = new ArrayList<>();
                // YOUR CODE HERE | | |
                //               \/ \/ \/
                tabs.add(new SwerveTab(swerveDrive));
                // STOP HERE OR DIE
                ShuffleboardInfo shuffleboardInfo = ShuffleboardInfo.getInstance();
                shuffleboardInfo.addTabs(tabs);

  }

  private void configureBindings() {
    //TODO: Do you need to negate the LeftY so that pointing forwards goes away from driver station?
    swerveDrive.setDefaultCommand(new FieldCentricDriveState(swerveDrive,
     () -> (MathUtils.deadband(driverController.getLeftX(), DriverConstants.JOYSTICK_DEADBAND)),
      () -> (-MathUtils.deadband(driverController.getLeftY(), DriverConstants.JOYSTICK_DEADBAND)),
      () -> (MathUtils.deadband(driverController.getRightX(), DriverConstants.JOYSTICK_DEADBAND))));

    driverController.a().onTrue(Commands.either(new InstantCommand(() -> swerveDrive.getCurrentCommand().cancel()),
     new LockWheelsState(swerveDrive), () -> (swerveDrive.getCurrentCommand() instanceof LockWheelsState)));

     driverController.leftStick().onTrue(Commands.either(
      new InstantCommand(() -> swerveDrive.setDefaultCommand(new AbsoluteDriveState(swerveDrive, 
      () -> (MathUtils.deadband(driverController.getLeftX(), DriverConstants.JOYSTICK_DEADBAND)),
      () -> (-MathUtils.deadband(driverController.getLeftY(), DriverConstants.JOYSTICK_DEADBAND)),
      () -> (MathUtils.deadband(driverController.getRightX(), DriverConstants.JOYSTICK_DEADBAND)),
      () -> (-MathUtils.deadband(driverController.getRightY(), DriverConstants.JOYSTICK_DEADBAND))))),
     new InstantCommand(() -> swerveDrive.setDefaultCommand(new FieldCentricDriveState(swerveDrive,
     () -> (MathUtils.deadband(driverController.getLeftX(), DriverConstants.JOYSTICK_DEADBAND)),
      () -> (MathUtils.deadband(driverController.getLeftY(), DriverConstants.JOYSTICK_DEADBAND)),
      () -> (MathUtils.deadband(driverController.getRightX(), DriverConstants.JOYSTICK_DEADBAND))))),
       () -> (swerveDrive.getCurrentCommand() instanceof FieldCentricDriveState)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void setDriveBaseBrake(boolean isBraked){
    swerveDrive.setMotorBrake(isBraked);
  }
}
