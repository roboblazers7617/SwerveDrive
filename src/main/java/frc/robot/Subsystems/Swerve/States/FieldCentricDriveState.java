// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve.States;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Subsystems.Swerve.SwerveDrive;
import frc.team4272.globals.State;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;
import frc.team4272.globals.MathUtils;
import frc.robot.Constants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.SwerveConstants;
/**This is a manual control state,
 *  where one joystick controls the field-relative translation and another controls  */
public class FieldCentricDriveState extends State<SwerveDrive> {
  /** Creates a new FieldCentricState. */
  private final Supplier<Double> vX, vY, vTheta;
  private ChassisSpeeds speeds;
  private Translation2d translation;
  public FieldCentricDriveState(SwerveDrive swerveDrive, Supplier<Double> vX, Supplier<Double> vY, Supplier<Double> vTheta) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(swerveDrive); 
    this.vX = vX;
    this.vY = vY;
    this.vTheta = vTheta;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speeds = requiredSubsystem.getTargetSpeeds(MathUtils.deadband(vX.get(), DriverConstants.joystickDeadband),
    MathUtils.deadband(vY.get(), DriverConstants.joystickDeadband),
    MathUtils.deadband(vTheta.get(), DriverConstants.joystickDeadband));

    translation = SwerveController.getTranslation2d(speeds);

    translation = SwerveMath.limitVelocity(translation, requiredSubsystem.getFieldVelocity(), requiredSubsystem.getPose(), 
    Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(SwerveConstants.DRIVEBASE), 
    requiredSubsystem.getSwerveDriveConfiguration());
    //TODO: do we want second order Kinematics?
    requiredSubsystem.drive(translation, speeds.omegaRadiansPerSecond, true, false);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}