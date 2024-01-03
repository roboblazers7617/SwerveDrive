// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.SwerveConstants;
//import frc.robot.Shuffleboard.DriverStationTab;
//import frc.robot.Shuffleboard.ShuffleboardInfo;
//import frc.robot.Shuffleboard.ShuffleboardTabBase;
//import frc.robot.Shuffleboard.SwerveTab;
import frc.robot.Subsystems.Swerve.SwerveDrive;
import frc.robot.Subsystems.Swerve.States.AbsoluteDriveState;
import frc.robot.Subsystems.Swerve.States.FieldCentricDriveState;
import frc.robot.Subsystems.Swerve.States.LockWheelsState;
import frc.team4272.globals.MathUtils;

public class RobotContainer {
  private SwerveDrive swerveDrive;
  private CommandXboxController driverController;
  public String path;

  private final FieldCentricDriveState fieldCentricDriveState;
  private final AbsoluteDriveState absoluteDriveState;

  public RobotContainer() {
    swerveDrive = new SwerveDrive();
    driverController = new CommandXboxController(DriverConstants.DRIVER_CONTROLLER_PORT);

    fieldCentricDriveState = new FieldCentricDriveState(swerveDrive,
    () -> (-MathUtils.deadband(driverController.getLeftY(), DriverConstants.JOYSTICK_DEADBAND)),
     () -> (-MathUtils.deadband(driverController.getLeftX(), DriverConstants.JOYSTICK_DEADBAND)),
     () -> (-MathUtils.deadband(driverController.getRightX(), DriverConstants.JOYSTICK_DEADBAND)),
     driverController.y(),
     driverController.a(),
     driverController.x(),
     driverController.b());

     absoluteDriveState  = (new AbsoluteDriveState(swerveDrive, 
     () -> (-MathUtils.deadband(driverController.getLeftY(), DriverConstants.JOYSTICK_DEADBAND)),
     () -> (-MathUtils.deadband(driverController.getLeftX(), DriverConstants.JOYSTICK_DEADBAND)),
     () -> (-MathUtils.deadband(driverController.getRightX(), DriverConstants.JOYSTICK_DEADBAND)),
     () -> (-MathUtils.deadband(driverController.getRightY(), DriverConstants.JOYSTICK_DEADBAND)),
     driverController.y(),
     driverController.a(),
     driverController.x(),
     driverController.b()));

    configureBindings();

   /*  ArrayList<ShuffleboardTabBase> tabs = new ArrayList<>();
                // YOUR CODE HERE |  |  |
                //               \/ \/ \/
                tabs.add(new SwerveTab(swerveDrive));
                tabs.add(new DriverStationTab(swerveDrive));
                // STOP HERE OR DIE
                ShuffleboardInfo shuffleboardInfo = ShuffleboardInfo.getInstance();
                shuffleboardInfo.addTabs(tabs);*/

    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    //StringLogEntry stringLog = new StringLogEntry(DataLogManager.getLog(), "/");
    //stringLog.append("testing");
    //stringLog.finish();
  }

  private void configureBindings() {
    //TODO: Do you need to negate the LeftY so that pointing forwards goes away from driver station?
    swerveDrive.setDefaultCommand(fieldCentricDriveState);


    driverController.povDown().toggleOnTrue(new LockWheelsState(swerveDrive));

     driverController.povLeft().onTrue(
     Commands.either(
      Commands.parallel(Commands.runOnce(() -> swerveDrive.setDefaultCommand(absoluteDriveState))
      .andThen(new ScheduleCommand(absoluteDriveState)),
      Commands.print(swerveDrive.getDefaultCommand().getName())), 
     Commands.parallel(Commands.runOnce(() -> swerveDrive.setDefaultCommand(fieldCentricDriveState), swerveDrive)
     .andThen(new ScheduleCommand(fieldCentricDriveState)),
     Commands.print(swerveDrive.getDefaultCommand().getName())),
      this::isFieldCentric));
      }
  

  public boolean isFieldCentric(){
    return swerveDrive.getDefaultCommand() instanceof FieldCentricDriveState;
  }
  
      public Command getAutonomousCommand() {
    //return Commands.print("No autonomous command configured");
        return Commands.sequence(new InstantCommand(() -> swerveDrive.drive(new Translation2d(.5, 0), 0, false, false)),
         Commands.waitSeconds(5),
          new InstantCommand(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false)));
  }

  public void setDriveBaseBrake(boolean isBraked){
    swerveDrive.setMotorBrake(isBraked);
  }
}
