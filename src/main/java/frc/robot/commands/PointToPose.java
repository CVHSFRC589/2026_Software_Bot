// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PointToPose extends Command {
  PIDController m_turningController;
  DriveSubsystem m_driveSubsystem;
  DoubleSupplier m_ySpeed;
  DoubleSupplier m_xSpeed;
  Pose2d m_targetPose = new Pose2d(10.728700637202202, 6.861022058101391, new Rotation2d(6.861022058101391));

  /** Creates a new PointToPose. */
  public PointToPose(DriveSubsystem driveSubsystem, DoubleSupplier ySpeed, DoubleSupplier xSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
    m_xSpeed = xSpeed;
    m_ySpeed = ySpeed;
    m_turningController = new PIDController(0.00625, 0, 0);
    
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
    // double pose_x = SmartDashboard.getNumber("Target Pose X", 0);
    // double pose_y = SmartDashboard.getNumber("Target Pose Y", 0);
    // double pose_a = SmartDashboard.getNumber("Target Pose A", 0);
    // m_targetPose = new Pose2d(pose_x, pose_y, new Rotation2d(pose_a));
    

    // getting robot pose for temp storage
    Pose2d pose = m_driveSubsystem.getPose();

    // getting x distance from target
    double x = m_targetPose.getMeasureX().magnitude() - pose.getMeasureX().magnitude();
    SmartDashboard.putNumber("X distance", x);
    // getting y distance from target
    double y = m_targetPose.getMeasureY().magnitude() - pose.getMeasureY().magnitude();
SmartDashboard.putNumber("Y distance", y);
    // getting angle required to point at target
    // double targetYaw = Units.radiansToDegrees(Math.atan(x/y));
    double targetYaw = Units.radiansToDegrees(Math.atan2(x,y)) + 180 - pose.getRotation().getDegrees();
    
SmartDashboard.putNumber("Target Yaw distance", targetYaw);
    // Auto-align when requested
    turn = 1.0 * MathUtil.applyDeadband(m_turningController.calculate(targetYaw), 0.01)
        * DriveConstants.kMaxAngularSpeed;
SmartDashboard.putNumber("Turn velocity", turn);
    // Command drivetrain motors based on target speeds
    m_driveSubsystem.drive(forward, strafe, turn, true);
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
