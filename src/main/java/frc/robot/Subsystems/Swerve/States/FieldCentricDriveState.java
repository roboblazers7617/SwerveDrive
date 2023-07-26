// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve.States;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Swerve.SwerveDrive;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;
import frc.team4272.globals.MathUtils;
import frc.robot.Constants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.SwerveConstants;
/**This is a manual control state,
 *  where one joystick controls the field-relative translation and another controls the rate of drivetrain rotation. 
 * This is opposed to the absolute drive, where the command is passed an absolue heading of the robot for it to face.*/

public class FieldCentricDriveState extends CommandBase {
  /** Creates a new FieldCentricState. */

  private final SwerveDrive swerveDrive;
  private final Supplier<Double> vX, vY, vTheta;
  private ChassisSpeeds speeds;
  private Translation2d translation;
  public FieldCentricDriveState(SwerveDrive swerveDrive, Supplier<Double> vX, Supplier<Double> vY, Supplier<Double> vTheta) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive); 
    this.swerveDrive = swerveDrive;
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

    speeds = swerveDrive.getTargetSpeeds(
    vX.get(),
    vY.get(),
    //This is in radians, we might need to add a speed conversion factor to turn faster if that is deemed nessecary
    vTheta.get());

    translation = SwerveController.getTranslation2d(speeds);

    translation = SwerveMath.limitVelocity(translation, swerveDrive.getFieldVelocity(), swerveDrive.getPose(), 
    Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(SwerveConstants.DRIVEBASE), 
    swerveDrive.getSwerveDriveConfiguration());
    swerveDrive.drive(translation, speeds.omegaRadiansPerSecond, true, false);
    
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
