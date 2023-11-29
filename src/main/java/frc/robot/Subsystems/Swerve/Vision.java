// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  PhotonCamera camera = new PhotonCamera(Constants.VisionConstants.PhotonCameraName);
  PhotonPipelineResult result;
  PhotonTrackedTarget bestTag;
  AprilTagFieldLayout layout;
  PhotonPoseEstimator poseEstimator;
  



  public Vision() {
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException e) {
      layout = null;
    }
    poseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP, camera,
       VisionConstants.CAMERA_POSITION);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     result = camera.getLatestResult();
    if (result.hasTargets())
     bestTag = result.getBestTarget();
   else
    bestTag = null;

  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    //poseEstimator.setLastPose(prevEstimatedRobotPose);
    return poseEstimator.update();
  }
  public double angleToTag(){
    if (bestTag != null)
      return bestTag.getYaw();
    else
      return 0;
  }

}