// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PointToAprilTag extends Command {
  /** Creates a new AprilTagPointToTarget. */
  DriveSubsystem m_driveSubsystem;
  int m_aprilTagId;
  DoubleSupplier m_ySpeed;
  DoubleSupplier m_xSpeed;
  PhotonCamera m_camera;
  PIDController m_turningController;

  public PointToAprilTag(DriveSubsystem driveSubsystem, int aprilTagId,
      DoubleSupplier ySpeed, DoubleSupplier xSpeed, PhotonCamera camera) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
    m_aprilTagId = aprilTagId;
    m_ySpeed = ySpeed;
    m_xSpeed = xSpeed;
    m_camera = camera;
    m_turningController = new PIDController(AutoConstants.kPThetaController, 0, 0);
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forward = m_ySpeed.getAsDouble() * DriveConstants.kMaxSpeedMetersPerSecond;
    double strafe = m_xSpeed.getAsDouble() * DriveConstants.kMaxSpeedMetersPerSecond;
    double turn = 0;
    m_turningController.setSetpoint(0.0);
    // Read in relevant data from the Camera
    boolean targetVisible = false;
    double targetYaw = 0.0;
    var results = m_camera.getAllUnreadResults();
    if (!results.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      var result = results.get(results.size() - 1);

      if (result.hasTargets()) {
        // At least one AprilTag was seen by the camera
        for (var target : result.getTargets()) {
          if (target.getFiducialId() == m_aprilTagId) {
            // Found Tag, record its information
            targetYaw = target.getYaw();
            // m_turningController.setSetpoint(target.getYaw());
            targetVisible = true;
          }
        }
      }
    }

    // Auto-align when requested
    if (targetVisible) {
      // Driver wants auto-alignment to tag 7
      // And, tag 7 is in sight, so we can turn toward it.
      // Override the driver's turn command with an automatic one that turns toward
      // the tag.
      
      turn = 1.0 * MathUtil.applyDeadband(m_turningController.calculate(targetYaw), 0.01) * DriveConstants.kMaxAngularSpeed;
      SmartDashboard.putNumber("PIDCalculate", m_turningController.calculate(targetYaw));
      SmartDashboard.putNumber("getYaw", m_turningController.getSetpoint());
      SmartDashboard.putNumber("xSpeed", strafe);
      SmartDashboard.putNumber("ySpeed", forward);
      SmartDashboard.putNumber("turn", turn);

    }

    // Command drivetrain motors based on target speeds
    m_driveSubsystem.drive(forward, strafe, turn, true);

    // Put debug information to the dashboard
    SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
