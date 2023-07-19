// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve.States;

import frc.robot.Subsystems.Swerve.SwerveDrive;
import frc.team4272.globals.State;
/**A state that locks the wheels of the swerve drive to force it to remain stationary */
public class LockWheelsState extends State<SwerveDrive> {
  /** Creates a new LockWheelsState. */
  public LockWheelsState(SwerveDrive swerveDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    //TODO: Does the call to super here addRequirements for the subsystem?
    super(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    requiredSubsystem.lock();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  //TODO: Doesn't this command need to end? 
  public boolean isFinished() {
    return false;
  }
}
