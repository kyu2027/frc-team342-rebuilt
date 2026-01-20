// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.*;
import frc.robot.AprilTagIDs.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import java.util.List;
import java.util.ArrayList;

public class PhotonVision extends SubsystemBase {

  private PhotonCamera leftTurretCamera;
  private PhotonCamera rightTurretCamera;
  private PhotonCamera leftRobotCamera;
  private PhotonCamera rightRobotCamera;

  private PhotonPoseEstimator leftTurretCameraPoseEstimator;
  private PhotonPoseEstimator rightTurretCameraPoseEstimator;
  private PhotonPoseEstimator leftRobotCameraPoseEstimator;
  private PhotonPoseEstimator rightRobotCameraPoseEstimator;
  private Pose3d robotPose;

  private List<PhotonPipelineResult> allCameraResults;
  private PhotonTrackedTarget trackedHubTag;

  /** Creates a new PhotonVision. */
  public PhotonVision() {

    leftTurretCamera = new PhotonCamera(LEFT_TURRET_CAMERA);
    rightTurretCamera = new PhotonCamera(RIGHT_TURRET_CAMERA);

    leftRobotCamera = new PhotonCamera(LEFT_ROBOT_CAMERA);
    rightRobotCamera = new PhotonCamera(RIGHT_ROBOT_CAMERA);

    allCameraResults = getAllCameraPipelines();

    leftTurretCameraPoseEstimator = new PhotonPoseEstimator(FIELD_LAYOUT, LEFT_TURRET_CAMERA_TRANSFORM_3D);
    rightTurretCameraPoseEstimator = new PhotonPoseEstimator(FIELD_LAYOUT, RIGHT_TURRET_CAMERA_TRANSFORM_3D);
    leftRobotCameraPoseEstimator = new PhotonPoseEstimator(FIELD_LAYOUT, LEFT_ROBOT_CAMERA_TRANSFORM_3D);
    rightRobotCameraPoseEstimator = new PhotonPoseEstimator(FIELD_LAYOUT, RIGHT_ROBOT_CAMERA_TRANSFORM_3D);

    robotPose = new Pose3d();
  }

  /**Checks if a tag is usable. If the tag pose is present, meets the ambiguity requirement, and is not too far, it is usable.
   * 
   * @param tag The tag to check
   */
  public boolean tagIsUsable(PhotonTrackedTarget tag) {
    return
      FIELD_LAYOUT.getTagPose(tag.getFiducialId()).isPresent() && 
      tag.getPoseAmbiguity() < AMBIGUITY_CUTOFF &&
      tag.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm() < TAG_CUTOFF_DISTANCE;
  }

  /**Calculates the Pose3d of the robot by averaging the Pose3d calculations from each camera using multiple unique tags.
   * 
   * @return The averaged Pose3d of the robot using multiple unique tags.
   */
  public Pose3d getRobotPoseWithMultiTags() {
    return new Pose3d(getAveragedXWithMultiTags(), getAveragedYWithMultiTags(), 0, new Rotation3d(0, 0, getAveragedYawWithMultiTags()));
  }

  /**Calculates the Pose3d of the robot by averaging the Pose3d calculations from each camera using a single tag.
   * 
   * @return The average Pose3d of the robot using a single tag.
   */
  public Pose3d getRobotPoseWithSingleTag() {
    return new Pose3d(getAveragedXWithSingleTag(), getAveragedXWithSingleTag(), 0, new Rotation3d(0, 0, getAveragedYawWithSingleTag()));
  }

  /**Returns the best PhotonPipelineResult from a singular camera.
   * 
   * @param camera The camera from which to pull the results.
   * @return The best PhotonPipelineResult from the given camera.
   */
  public PhotonPipelineResult getPhotonPipeline(PhotonCamera camera) {
    return camera.getAllUnreadResults().get(0);
  }

  /**Returns all tags seen by all cameras.
   * 
   * @return The list of tags seen.
   */
  public List<PhotonTrackedTarget> getAllTagsSeen() {
    List<PhotonTrackedTarget> tags = new ArrayList<>();
    for(PhotonPipelineResult result : allCameraResults) {
      for(PhotonTrackedTarget tag : result.getTargets()) {
        if(tagIsUsable(tag)) {
          tags.add(tag);
        }
      }
    }

    return tags;
  }

  /**Returns the requested tag.
   * 
   * @param tagID The ID of the tag to look for.
   * @return The requested tag.
   */
  public PhotonTrackedTarget getSpecificTag(double tagID) {
    List<PhotonTrackedTarget> wantedTags = new ArrayList<>();

    for(PhotonTrackedTarget tag : getAllTagsSeen()) {
      if(tag.getFiducialId() == tagID) {
        wantedTags.add(tag);
      }
    }

    PhotonTrackedTarget firstTag = wantedTags.get(0);
    PhotonTrackedTarget previousTag = wantedTags.get(0);
    PhotonTrackedTarget bestTag = wantedTags.get(0);
    for(PhotonTrackedTarget tag : wantedTags) {
      if(tag.getFiducialId() != firstTag.getFiducialId()) {
        if(tag.getPoseAmbiguity() < previousTag.getPoseAmbiguity()) {
          bestTag = tag;
        }
      }
      previousTag = tag;
    }

    return bestTag;
  }

  /**Sets the hub tag to track.
   * 
   * @param tag The hub tag to track.
   */
  public void setTrackedHubTag(PhotonTrackedTarget tag) {
    trackedHubTag = tag;
  }

  /**Checks to see if an alliance hub tag is present.
   * 
   * @return {@code true} if an alliance hub tag is present, {@code false} otherwise.
   */
  public boolean allianceHubTagIsPresent() {
    for(PhotonPipelineResult result : allCameraResults) {
      for(PhotonTrackedTarget tag : result.getTargets()) {
        if(tagIsUsable(tag)) {
          for(hubTagsIDs tagID : hubTagsIDs.values()) {
            for(int i = 0; i < hubTagsIDs.values().length; i++) {
              if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                if(tag.getFiducialId() == tagID.getRedHubTagID()) {
                  setTrackedHubTag(tag);
                  return true;
                }
              }else{
                if(tag.getFiducialId() == tagID.getBlueHubTagID()) {
                  setTrackedHubTag(tag);
                  return true;
                }
              }
            }
          }
        }
      }
    }

    return false;
  }

  /**Checks to see if an alliance outpost tag is present.
   * 
   * @return {@code true} if an alliance outpost tag is present, {@code false} otherwise.
   */
  public boolean allianceOutpostTagIsPresent() {
    for(PhotonPipelineResult result : allCameraResults) {
      for(PhotonTrackedTarget tag : result.getTargets()) {
        if(tagIsUsable(tag)) {
          for(outpostTagsIDs tagID : outpostTagsIDs.values()) {
            for(int i = 0; i < outpostTagsIDs.values().length; i++) {
              if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                if(tag.getFiducialId() == tagID.getRedOutpostTagID()) {
                  return true;
                }
              }else{
                if(tag.getFiducialId() == tagID.getBlueOutpostTagID()) {
                  return true;
                }
              }
            }
          }
        }
      }
    }

    return false;
  }

  /**Checks to see if an alliance tower tag is present.
   * 
   * @return {@code true} if an alliance tower tag is present, {@code false} otherwise.
   */
  public boolean allianceTowerTagIsPresent() {
    for(PhotonPipelineResult result : allCameraResults) {
      for(PhotonTrackedTarget tag : result.getTargets()) {
        if(tagIsUsable(tag)) {
          for(towerTagsIDs tagID : towerTagsIDs.values()) {
            for(int i = 0; i < towerTagsIDs.values().length; i++) {
              if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                if(tag.getFiducialId() == tagID.getRedTowerTagID()) {
                  return true;
                }
              }else{
                if(tag.getFiducialId() == tagID.getBlueTowerTagID()) {
                  return true;
                }
              }
            }
          }
        }
      }
    }

    return false;
  }

  /**Checks if there is a tag present in any camera.
   * 
   * @return {@code true} if there is a tag, {@code false} if not.
   */
  public boolean tagIsPresentAcrossAllCameras() {
    for(PhotonPipelineResult result : allCameraResults) {
      for(PhotonTrackedTarget tag : result.getTargets()) {
        if(tagIsUsable(tag)) {
          return true;
        }
      }
    }
    return false;
  }

  /**Checks if there is a tag present in a specific camera.
   * 
   * @param camera The camera to check.
   * @return {@code true} if there is a tag, {@code false} if not.
   */
  public boolean tagIsPresentInCamera(PhotonCamera camera) {
    for(PhotonPipelineResult result : camera.getAllUnreadResults()) {
      for(PhotonTrackedTarget tag : result.getTargets()) {
        if(tagIsUsable(tag)) {
          return true;
        }
      }
    }

    return false;
  }

  /**Get the yaw to the given tag.
   * 
   * @param tag The tag to get the yaw to.
   * @return The yaw to the given tag.
   */
  public double getHorizontalAngleToTarget(PhotonTrackedTarget tag) {
    return tag.getYaw();
  }

  /**Get the pitch to the given tag.
   * 
   * @param tag The tag to get the pitch to.
   * @return The pitch to the given tag.
   */
  public double getVerticalAngleToTarget(PhotonTrackedTarget tag) {
    return tag.getPitch();
  }

  /**Get the PhotonPipelineResults from all the cameras.
   * 
   * @return The list of PhotonPipelineResults.
   */
  public List<PhotonPipelineResult> getAllCameraPipelines() {
    List<PhotonPipelineResult> allCamResults = new ArrayList<>();
    List<PhotonPipelineResult> leftTurretResult = new ArrayList<>();
    List<PhotonPipelineResult> rightTurretResult = new ArrayList<>();
    List<PhotonPipelineResult> leftRobotResult = new ArrayList<>();
    List<PhotonPipelineResult> rightRobotResult = new ArrayList<>();

    leftTurretResult = leftTurretCamera.getAllUnreadResults();
    rightTurretResult = rightTurretCamera.getAllUnreadResults();
    leftRobotResult = leftRobotCamera.getAllUnreadResults();
    rightRobotResult = rightRobotCamera.getAllUnreadResults();

    allCamResults.add(leftTurretResult.get(leftTurretResult.size() - 1));
    allCamResults.add(rightTurretResult.get(rightTurretResult.size() - 1));
    allCamResults.add(leftRobotResult.get(leftRobotResult.size() - 1));
    allCamResults.add(rightRobotResult.get(rightRobotResult.size() - 1));

    return allCamResults;
  }

  /**Get the average X position from Pose3d calculations across all cameras using multiple unique tags.
   * 
   * @return The average X position.
   */
  public double getAveragedXWithMultiTags() {
    double numberOfCamsWithTags = 0;

    double leftTurretCameraX = 0;
    double rightTurretCameraX = 0;
    double leftRobotCameraX = 0;
    double rightRobotCameraX = 0;

    if(tagIsPresentInCamera(leftTurretCamera)) {
      leftTurretCameraX = leftTurretCameraPoseEstimator.estimateCoprocMultiTagPose(getPhotonPipeline(leftTurretCamera)).get().estimatedPose.getX();
      numberOfCamsWithTags++;
    }

    if(tagIsPresentInCamera(rightTurretCamera)) {
      rightTurretCameraX = rightTurretCameraPoseEstimator.estimateCoprocMultiTagPose(getPhotonPipeline(rightTurretCamera)).get().estimatedPose.getX();
      numberOfCamsWithTags++;
    }

    if(tagIsPresentInCamera(leftRobotCamera)) {
      leftRobotCameraX = leftRobotCameraPoseEstimator.estimateCoprocMultiTagPose(getPhotonPipeline(leftRobotCamera)).get().estimatedPose.getX();
      numberOfCamsWithTags++;
    }

    if(tagIsPresentInCamera(rightRobotCamera)) {
      rightRobotCameraX = rightRobotCameraPoseEstimator.estimateCoprocMultiTagPose(getPhotonPipeline(rightRobotCamera)).get().estimatedPose.getX();
      numberOfCamsWithTags++;
    }

    return (leftTurretCameraX + rightTurretCameraX + leftRobotCameraX + rightRobotCameraX) / numberOfCamsWithTags;
  }

  /**Get the average Y position from Pose3d calculations across all cameras using multiple unique tags.
   * 
   * @return The average Y position.
   */
  public double getAveragedYWithMultiTags() {
    double numberOfCamsWithTags = 0;

    double leftTurretCameraY = 0;
    double rightTurretCameraY = 0;
    double leftRobotCameraY = 0;
    double rightRobotCameraY = 0;

    if(tagIsPresentInCamera(leftTurretCamera)) {
      leftTurretCameraY = leftTurretCameraPoseEstimator.estimateCoprocMultiTagPose(getPhotonPipeline(leftTurretCamera)).get().estimatedPose.getY();
      numberOfCamsWithTags++;
    }

    if(tagIsPresentInCamera(rightTurretCamera)) {
      rightTurretCameraY = rightTurretCameraPoseEstimator.estimateCoprocMultiTagPose(getPhotonPipeline(rightTurretCamera)).get().estimatedPose.getY();
      numberOfCamsWithTags++;
    }

    if(tagIsPresentInCamera(leftRobotCamera)) {
      leftRobotCameraY = leftRobotCameraPoseEstimator.estimateCoprocMultiTagPose(getPhotonPipeline(leftRobotCamera)).get().estimatedPose.getY();
      numberOfCamsWithTags++;
    }

    if(tagIsPresentInCamera(rightRobotCamera)) {
      rightRobotCameraY = rightRobotCameraPoseEstimator.estimateCoprocMultiTagPose(getPhotonPipeline(rightRobotCamera)).get().estimatedPose.getY();
      numberOfCamsWithTags++;
    }

    return (leftTurretCameraY + rightTurretCameraY + leftRobotCameraY + rightRobotCameraY) / numberOfCamsWithTags;
  }

  /**Get the average yaw from Pose3d calculations across all cameras using multiple unique tags.
   * 
   * @return The average yaw.
   */
  public double getAveragedYawWithMultiTags() {
    double numberOfCamsWithTags = 0;

    double leftTurretCameraYaw = 0;
    double rightTurretCameraYaw = 0;
    double leftRobotCameraYaw = 0;
    double rightRobotCameraYaw = 0;

    if(tagIsPresentInCamera(leftTurretCamera)) {
      leftTurretCameraYaw = leftTurretCameraPoseEstimator.estimateCoprocMultiTagPose(getPhotonPipeline(leftTurretCamera)).get().estimatedPose.getRotation().getAngle();
      numberOfCamsWithTags++;
    }

    if(tagIsPresentInCamera(rightTurretCamera)) {
      rightTurretCameraYaw = rightTurretCameraPoseEstimator.estimateCoprocMultiTagPose(getPhotonPipeline(rightTurretCamera)).get().estimatedPose.getRotation().getAngle();
      numberOfCamsWithTags++;
    }

    if(tagIsPresentInCamera(leftRobotCamera)) {
      leftRobotCameraYaw = leftRobotCameraPoseEstimator.estimateCoprocMultiTagPose(getPhotonPipeline(leftRobotCamera)).get().estimatedPose.getRotation().getAngle();
      numberOfCamsWithTags++;
    }

    if(tagIsPresentInCamera(rightRobotCamera)) {
      rightRobotCameraYaw = rightRobotCameraPoseEstimator.estimateCoprocMultiTagPose(getPhotonPipeline(rightRobotCamera)).get().estimatedPose.getRotation().getAngle();
      numberOfCamsWithTags++;
    }

    return ((leftTurretCameraYaw + rightTurretCameraYaw + leftRobotCameraYaw + rightRobotCameraYaw) / numberOfCamsWithTags) * (180/Math.PI);
  }

  /**Get the average X position from Pose3d calculations across all cameras using a single tag.
   * 
   * @return The average X position.
   */
  public double getAveragedXWithSingleTag() {
    double numberOfCamsWithTags = 0;

    double leftTurretCameraX = 0;
    double rightTurretCameraX = 0;
    double leftRobotCameraX = 0;
    double rightRobotCameraX = 0;

    if(tagIsPresentInCamera(leftTurretCamera)) {
      leftTurretCameraX = leftTurretCameraPoseEstimator.estimateLowestAmbiguityPose(getPhotonPipeline(leftTurretCamera)).get().estimatedPose.getX();
      numberOfCamsWithTags++;
    }

    if(tagIsPresentInCamera(rightTurretCamera)) {
      rightTurretCameraX = rightTurretCameraPoseEstimator.estimateLowestAmbiguityPose(getPhotonPipeline(rightTurretCamera)).get().estimatedPose.getX();
      numberOfCamsWithTags++;
    }

    if(tagIsPresentInCamera(leftRobotCamera)) {
      leftRobotCameraX = leftRobotCameraPoseEstimator.estimateLowestAmbiguityPose(getPhotonPipeline(leftRobotCamera)).get().estimatedPose.getX();
      numberOfCamsWithTags++;
    }

    if(tagIsPresentInCamera(rightRobotCamera)) {
      rightRobotCameraX = rightRobotCameraPoseEstimator.estimateLowestAmbiguityPose(getPhotonPipeline(rightRobotCamera)).get().estimatedPose.getX();
      numberOfCamsWithTags++;
    }

    return (leftTurretCameraX + rightTurretCameraX + leftRobotCameraX + rightRobotCameraX) / numberOfCamsWithTags;
  }

  /**Get the average Y position from Pose3d calculations across all cameras using a single tag.
   * 
   * @return The average Y position.
   */
  public double getAveragedYWithSingleTag() {
    double numberOfCamsWithTags = 0;

    double leftTurretCameraY = 0;
    double rightTurretCameraY = 0;
    double leftRobotCameraY = 0;
    double rightRobotCameraY = 0;

    if(tagIsPresentInCamera(leftTurretCamera)) {
      leftTurretCameraY = leftTurretCameraPoseEstimator.estimateLowestAmbiguityPose(getPhotonPipeline(leftTurretCamera)).get().estimatedPose.getY();
      numberOfCamsWithTags++;
    }

    if(tagIsPresentInCamera(rightTurretCamera)) {
      rightTurretCameraY = rightTurretCameraPoseEstimator.estimateLowestAmbiguityPose(getPhotonPipeline(rightTurretCamera)).get().estimatedPose.getY();
      numberOfCamsWithTags++;
    }

    if(tagIsPresentInCamera(leftRobotCamera)) {
      leftRobotCameraY = leftRobotCameraPoseEstimator.estimateLowestAmbiguityPose(getPhotonPipeline(leftRobotCamera)).get().estimatedPose.getY();
      numberOfCamsWithTags++;
    }

    if(tagIsPresentInCamera(rightRobotCamera)) {
      rightRobotCameraY = rightRobotCameraPoseEstimator.estimateLowestAmbiguityPose(getPhotonPipeline(rightRobotCamera)).get().estimatedPose.getY();
      numberOfCamsWithTags++;
    }

    return (leftTurretCameraY + rightTurretCameraY + leftRobotCameraY + rightRobotCameraY) / numberOfCamsWithTags;
  }

  /**Get the average yaw from Pose3d calculations across all cameras using a single tag.
   * 
   * @return The average yaw.
   */
  public double getAveragedYawWithSingleTag() {
    double numberOfCamsWithTags = 0;

    double leftTurretCameraYaw = 0;
    double rightTurretCameraYaw = 0;
    double leftRobotCameraYaw = 0;
    double rightRobotCameraYaw = 0;

    if(tagIsPresentInCamera(leftTurretCamera)) {
      leftTurretCameraYaw = leftTurretCameraPoseEstimator.estimateLowestAmbiguityPose(getPhotonPipeline(leftTurretCamera)).get().estimatedPose.getRotation().getAngle();
      numberOfCamsWithTags++;
    }

    if(tagIsPresentInCamera(rightTurretCamera)) {
      rightTurretCameraYaw = rightTurretCameraPoseEstimator.estimateLowestAmbiguityPose(getPhotonPipeline(rightTurretCamera)).get().estimatedPose.getRotation().getAngle();
      numberOfCamsWithTags++;
    }

    if(tagIsPresentInCamera(leftRobotCamera)) {
      leftRobotCameraYaw = leftRobotCameraPoseEstimator.estimateLowestAmbiguityPose(getPhotonPipeline(leftRobotCamera)).get().estimatedPose.getRotation().getAngle();
      numberOfCamsWithTags++;
    }

    if(tagIsPresentInCamera(rightRobotCamera)) {
      rightRobotCameraYaw = rightRobotCameraPoseEstimator.estimateLowestAmbiguityPose(getPhotonPipeline(rightRobotCamera)).get().estimatedPose.getRotation().getAngle();
      numberOfCamsWithTags++;
    }

    return ((leftTurretCameraYaw + rightTurretCameraYaw + leftRobotCameraYaw + rightRobotCameraYaw) / numberOfCamsWithTags) * (180/Math.PI);
  }

  /**Gets the Pose3d of the robot.
   * 
   * @return The Pose3d of the robot.
   */
  public Pose3d getRobotPose3d() {
    return robotPose;
  }

  /**Sets the Pose3d of the robot.
   * 
   * @param newPose3d The new Pose3d of the robot.
   */
  public void setRobotPose3d(Pose3d newPose3d) {
    robotPose = newPose3d;
  }

  /**Updates the Pose3d of the robot.
   * 
   */
  public void updateRobotPose3d() {
    List<PhotonTrackedTarget> uniqueTags = new ArrayList<>();

    for(PhotonPipelineResult result : allCameraResults) {
      PhotonTrackedTarget firstTarget = result.getTargets().get(0);
      PhotonTrackedTarget previousTarget = result.getTargets().get(0);
      for(PhotonTrackedTarget target : result.getTargets()) {
        if(tagIsUsable(target)) {
          if(target.getFiducialId() != firstTarget.getFiducialId()) {
            if(target.getFiducialId() != previousTarget.getFiducialId()) {
              uniqueTags.add(target);
            }
          }
        }
        previousTarget = target;
      }
    }

    PhotonTrackedTarget firstTag = uniqueTags.get(0);
    PhotonTrackedTarget previousTag = uniqueTags.get(0);
    for (PhotonTrackedTarget tag : uniqueTags) {
      if(tag.getFiducialId() != firstTag.getFiducialId()) {
        if(tag.getFiducialId() != previousTag.getFiducialId()) {}
      }
      previousTag = tag;
      uniqueTags.remove(tag);
    }

    if(uniqueTags.size() <= 1) {
      getRobotPoseWithSingleTag();
      robotPose = getRobotPoseWithSingleTag();
    }else{
      getRobotPoseWithMultiTags();
      robotPose = getRobotPoseWithMultiTags();
    }
  }

  /**Gets the X position of the robot.
   * 
   * @return The X position of the robot.
   */
  public double getRobotX() {
    return getRobotPose3d().getX();
  }

  /**Gets the Y position of the robot.
   * 
   * @return The Y position of the robot.
   */
  public double getRobotY() {
    return getRobotPose3d().getY();
  }

  /**Gets the distance to the given tag.
   * 
   * @param tag The tag to get the distance to.
   * @return The distance to the given tag.
   */
  public double getDistanceToTag(PhotonTrackedTarget tag) {
    double xDistance = Math.abs(FIELD_LAYOUT.getTagPose(tag.getFiducialId()).get().getX() - getRobotX());
    double yDistance = Math.abs(FIELD_LAYOUT.getTagPose(tag.getFiducialId()).get().getY() - getRobotY());

    return Math.abs(Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2)));
  }

  /**Gets the distance to the alliance hub.
   * 
   * @return The distance to the alliance hub.
   */
  public double getDistanceToAllianceHub() {
    return getDistanceToTag(trackedHubTag);
  }

  //Puts data for vision on Elastic
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.setSmartDashboardType("PhotonVision");

    builder.addBooleanProperty("Alliance Hub Tag Seen", () -> allianceHubTagIsPresent(), null);
    builder.addBooleanProperty("Alliance Tower Tag Seen", () -> allianceTowerTagIsPresent(), null);
    builder.addBooleanProperty("Alliance Outpost Tag Seen", () -> allianceOutpostTagIsPresent(), null);
    builder.addBooleanProperty("Tag Seen", () -> tagIsPresentAcrossAllCameras(), null);
    builder.addDoubleProperty("Robot X", () -> getRobotX(), null);
    builder.addDoubleProperty("Robot Y", () -> getRobotY(), null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //Constantly updates the results from the cameras
    allCameraResults = getAllCameraPipelines();
    //Constantly updates the robot pose
    updateRobotPose3d();
    //Constantly checks to see if the alliance hub tag is present
    //If present, constantly get the angle and distance to the alliance hub
    if(allianceHubTagIsPresent()) {
      getHorizontalAngleToTarget(trackedHubTag);
      getDistanceToAllianceHub();
    }
  }
}
