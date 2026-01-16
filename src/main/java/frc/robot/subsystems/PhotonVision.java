// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class PhotonVision extends SubsystemBase {

  private PhotonCamera leftTurretCamera;
  private PhotonCamera rightTurretCamera;
  private PhotonCamera leftRobotCamera;
  private PhotonCamera rightRobotCamera;

  private PhotonPoseEstimator poseEstimator;

  /** Creates a new PhotonVision. */
  public PhotonVision() {

    leftTurretCamera = new PhotonCamera(LEFT_TURRET_CAMERA);
    rightTurretCamera = new PhotonCamera(RIGHT_TURRET_CAMERA);

    leftRobotCamera = new PhotonCamera(LEFT_ROBOT_CAMERA);
    rightRobotCamera = new PhotonCamera(RIGHT_ROBOT_CAMERA);

    poseEstimator = new PhotonPoseEstimator(
      FIELD_LAYOUT,
      new Transform3d(
        new Translation3d(
          0,
          0,
          0
        ),
        new Rotation3d(
          0,
          0, 
          0
        )
      )
    );

  }

  public boolean tagIsUsable(PhotonTrackedTarget tag) {
    return
      FIELD_LAYOUT.getTagPose(tag.getFiducialId()).isPresent() && 
      tag.getPoseAmbiguity() < AMBIGUITY_CUTOFF &&
      tag.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm() < TAG_CUTOFF_DISTANCE;
  }

  public Optional<EstimatedRobotPose> getRobotPoseWithMultiTags(PhotonPipelineResult result) {
    return poseEstimator.estimateCoprocMultiTagPose(result);
  }

  public Optional<EstimatedRobotPose> getRobotPoseWithSingleTag(PhotonPipelineResult result) {
    return poseEstimator.estimateLowestAmbiguityPose(result);
  }

  public PhotonPipelineResult getPhotonPipelineResult(PhotonCamera camera) {
    List<PhotonPipelineResult> result = camera.getAllUnreadResults();
    return result.get(result.size() - 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
