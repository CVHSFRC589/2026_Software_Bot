// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
  // Pose2d m_targetPose = new Pose2d(Units.inchesToMeters(468.56), Units.inchesToMeters(158.32), new Rotation2d(0));
  Pose2d m_targetPose = new Pose2d(Units.inchesToMeters(470.59), Units.inchesToMeters(25.37), new Rotation2d(0));
  double m_theta;

  /** Creates a new PointToPose. */
  public PointToPose(DriveSubsystem driveSubsystem, DoubleSupplier ySpeed, DoubleSupplier xSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
    m_xSpeed = xSpeed;
    m_ySpeed = ySpeed;
    m_turningController = new PIDController(0.0625, 1e-4, 0);

    addRequirements(m_driveSubsystem);
  }

  public double getAlignmentAngle(Pose2d pose1, Pose2d pose2) {
    // getting x distance from target
    double x = pose1.getX() - pose2.getX();
    SmartDashboard.putNumber("X distance", x);
    // getting y distance from target
    double y = pose1.getY() - pose2.getY();
    SmartDashboard.putNumber("Y distance", y);

    double thetaOld = Units.radiansToDegrees(Math.atan2(y, x));
    SmartDashboard.putNumber("thetaOld", thetaOld);
    double theta = Units.radiansToDegrees(Math.atan2(Math.abs(y), Math.abs(x)));
    // if (y == 0 && x > 0) {
    //   theta = 0;
    // } else if ( y == 0 && x < 0) {
    //   theta = -180;
    // }
    if (y > 0 && x < 0) {
      // quadrant 3
      theta = -1 * theta + 180;
    } else if (y < 0 && x < 0) {
      // quadrant 1
      theta -= 180;
    } else if (y < 0 && x > 0) {
      // quadrant 2
      theta *= -1;
    }
    return theta;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // getting robot pose for temp storage
    Pose2d pose = m_driveSubsystem.getPose();

    // // getting x distance from target
    // double x = m_targetPose.getX() - pose.getX();
    // // SmartDashboard.putNumber("X distance", x);
    // // getting y distance from target
    // double y = m_targetPose.getY() - pose.getY();

    // m_theta = Units.radiansToDegrees(Math.atan2(y, x));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forward = m_ySpeed.getAsDouble() * DriveConstants.kMaxSpeedMetersPerSecond;
    double strafe = m_xSpeed.getAsDouble() * DriveConstants.kMaxSpeedMetersPerSecond;
    double turn = 0;
    // m_turningController.setSetpoint(0.0);
    // double pose_x = SmartDashboard.getNumber("Target Pose X", 0);
    // double pose_y = SmartDashboard.getNumber("Target Pose Y", 0);
    // double pose_a = SmartDashboard.getNumber("Target Pose A", 0);
    // m_targetPose = new Pose2d(pose_x, pose_y, new Rotation2d(pose_a));

    // getting robot pose for temp storage
    Pose2d pose = m_driveSubsystem.getPose();

    // getting x distance from target
    // double x;
    // // getting y distance from target
    // double y;

    double theta = getAlignmentAngle(m_targetPose, pose);
    double phi = pose.getRotation().getDegrees();
    double error = theta - phi;

    if (error >= 180){
      error = 180 - error;
    }
    else if (error <= -180) {
      error = 360 + error;
    }
    SmartDashboard.putNumber("theta", theta);

    // Optional<Alliance> alliance = DriverStation.getAlliance();
    // if (alliance.get() == Alliance.Red) {
    // if (x < 0) {
    // targetYaw = (180 - theta) - phi;
    // } else {
    // targetYaw = theta - phi;

    // }
    // } else if (alliance.get() == Alliance.Blue) {
    // if (x > 0) {
    // targetYaw = (180 - theta) - phi;
    // } else {
    // targetYaw = theta - phi;
    // }
    // } else {

    // }

    // targetYaw -= 72;

    SmartDashboard.putNumber("Pose Point Target Yaw", theta);
    SmartDashboard.putNumber("phi", phi);
    SmartDashboard.putNumber("error", error);
    // if (x > 0) {
    // targetYaw = (180 - theta) - phi;
    // } else {
    // targetYaw = theta - phi;
    // }
    // SmartDashboard.putNumber("Y distance", y);
    // getting angle required to point at target
    // double targetYaw = Units.radiansToDegrees(Math.atan(x/y));
    // double targetYaw = Units.radiansToDegrees(Math.atan2(x,y)) + 180 -
    // pose.getRotation().getDegrees();
    // Auto-align when requested
    // m_turningController.setSetpoint(theta);
    m_turningController.setSetpoint(0);
    turn = -1.0 * MathUtil.applyDeadband(m_turningController.calculate(error), 0.00)
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
