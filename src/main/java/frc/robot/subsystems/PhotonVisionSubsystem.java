// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.FieldConstants;

public class PhotonVisionSubsystem extends SubsystemBase {
  /** Creates a new PhotonVisionSubsystem. */

  private final PhotonCamera m_topFrontCamera = new PhotonCamera("Front_Camera");
  private final PhotonCamera m_topBackCamera = new PhotonCamera("Back_Camera_Fisheye");

  private final PhotonPoseEstimator m_photonPoseEstimatorFront;
  private final PhotonPoseEstimator m_photonPoseEstimatorBack;
  private EstimateConsumer m_estimateConsumer;

  private Pose2d m_estimatedPose = new Pose2d();
  private Matrix<N3, N1> curStdDevs;

  public PhotonVisionSubsystem(boolean isHome, EstimateConsumer estimateConsumer) {
    m_photonPoseEstimatorFront = new PhotonPoseEstimator(FieldConstants.LoadLayout(isHome),
        CameraConstants.kRobotToFrontCam);
    m_photonPoseEstimatorBack = new PhotonPoseEstimator(FieldConstants.LoadLayout(isHome),
        CameraConstants.kRobotToBackCam);
    m_estimateConsumer = estimateConsumer;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // m_poseEstimator.estimateCoprocMultiTagPose(m_topFrontCamera.getAllUnreadResults().get(0));
    estimatePose2d();
  }

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates
   * dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from
   * the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets       All targets in this camera frame
   */
  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = Constants.CameraConstants.kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = Constants.CameraConstants.kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an
      // average-distance metric
      for (var tgt : targets) {
        var tagPose = m_photonPoseEstimatorFront.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty())
          continue;
        numTags++;
        avgDist += tagPose
            .get()
            .toPose2d()
            .getTranslation()
            .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        curStdDevs = Constants.CameraConstants.kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1)
          estStdDevs = Constants.CameraConstants.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        curStdDevs = estStdDevs;
      }
    }
  }

  /**
   * Returns the latest standard deviations of the estimated pose from {@link
   * #getEstimatedGlobalPose()}, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
   * SwerveDrivePoseEstimator}. This should
   * only be used when there are targets visible.
   */
  public Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }

  public PhotonCamera getTopFrontCamera() {
    return m_topFrontCamera;
  }

  public PhotonCamera getTopBackCamera() {
    return m_topBackCamera;
  }

  public void estimatePose2d() {
    List<PhotonPipelineResult> front_results = m_topFrontCamera.getAllUnreadResults();
    List<PhotonPipelineResult> back_results = m_topBackCamera.getAllUnreadResults();
    Optional<EstimatedRobotPose> visionEstimate = Optional.empty();
    for (var result : front_results) {
      visionEstimate = m_photonPoseEstimatorFront.estimateCoprocMultiTagPose(result);
      if (visionEstimate.isEmpty()) {
        visionEstimate = m_photonPoseEstimatorFront.estimateLowestAmbiguityPose(result);
      }
      updateEstimationStdDevs(visionEstimate, result.getTargets());
      // estimator.addVisionMeasurement(m_photonPoseEstimator.estimateCoprocMultiTagPose(result).get().estimatedPose.toPose2d(),
      // Timer.getFPGATimestamp());
      visionEstimate.ifPresent(
          est -> {
            // Change our trust in the measurement based on the tags we can see
            var estStdDevs = getEstimationStdDevs();

            m_estimateConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
          });
    }

    for (var result : back_results) {
      visionEstimate = m_photonPoseEstimatorBack.estimateCoprocMultiTagPose(result);
      if (visionEstimate.isEmpty()) {
        visionEstimate = m_photonPoseEstimatorBack.estimateLowestAmbiguityPose(result);
      }
      updateEstimationStdDevs(visionEstimate, result.getTargets());
      // estimator.addVisionMeasurement(m_photonPoseEstimator.estimateCoprocMultiTagPose(result).get().estimatedPose.toPose2d(),
      // Timer.getFPGATimestamp());
      visionEstimate.ifPresent(
          est -> {
            // Change our trust in the measurement based on the tags we can see
            var estStdDevs = getEstimationStdDevs();

            m_estimateConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
          });
    }
  }

  @FunctionalInterface
  public static interface EstimateConsumer {
    public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
  }
}
