// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.FieldConstants;

public class PhotonVisionSubsystem extends SubsystemBase {
  /** Creates a new PhotonVisionSubsystem. */

  private final PhotonCamera m_topFrontCamera = new PhotonCamera("SoftwareBot_2026_Camera_Top_Front");

  private final PhotonPoseEstimator m_poseEstimator;

  private Pose2d m_estimatedPose = new Pose2d();

  public PhotonVisionSubsystem(boolean isHome) {
    m_poseEstimator = new PhotonPoseEstimator(FieldConstants.LoadLayout(isHome), CameraConstants.kRobotToCam);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_poseEstimator.estimateCoprocMultiTagPose(m_topFrontCamera.getAllUnreadResults().get(0));

  }

  public PhotonCamera getTopFrontCamera() {
    return m_topFrontCamera;
  }

  public void estimatePose2d(SwerveDrivePoseEstimator estimator) {
    List<PhotonPipelineResult> results = m_topFrontCamera.getAllUnreadResults();
    for (var result : results) {
      estimator.addVisionMeasurement(m_poseEstimator.estimateCoprocMultiTagPose(result).get().estimatedPose.toPose2d(),
          Timer.getFPGATimestamp());
    }
  }
}
