// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static final class DriveConstants {
		// Driving Parameters - Note that these are not the maximum capable speeds of
		// the robot, rather the allowed maximum speeds
		public static final double kMaxSpeedMetersPerSecond = 4.8 / 8; // old is 4.8
		public static final double kMaxAngularSpeed = 2 * Math.PI / 8; // radians per second // old is 2 * Math.PI

		// Chassis configuration
		// public static final double kTrackWidth = Units.inchesToMeters(26.5);
		public static final double kTrackWidth = Units.inchesToMeters(23.5);
		// Distance between centers of right and left wheels on robot
		// public static final double kWheelBase = Units.inchesToMeters(26.5);
		public static final double kWheelBase = Units.inchesToMeters(23.5);
		// Distance between front and back wheels on robot
		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

		// Angular offsets of the modules relative to the chassis in radians
		public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
		public static final double kFrontRightChassisAngularOffset = 0;
		public static final double kBackLeftChassisAngularOffset = Math.PI;
		public static final double kBackRightChassisAngularOffset = Math.PI / 2;

		// SPARK MAX CAN IDs
		public static final int kFrontLeftDrivingCanId = 10;
		public static final int kRearLeftDrivingCanId = 30;
		public static final int kFrontRightDrivingCanId = 20;
		public static final int kRearRightDrivingCanId = 40;

		public static final int kFrontLeftTurningCanId = 11;
		public static final int kRearLeftTurningCanId = 31;
		public static final int kFrontRightTurningCanId = 21;
		public static final int kRearRightTurningCanId = 41;

		public static final boolean kGyroReversed = true;
		public static final int kPigeonCanID = 60;

		// PID
		public static final double kRotationalPIDkP = 0.03125;
		public static final double kRotationalPIDkI = 1e-5;
		public static final double kRotationalPIDkD = 0;
		public static final double kRotationalDeadband = 0.01;
	}

	public static final class ModuleConstants {
		// The MAXSwerve module can be configured with one of three pinion gears: 12T,
		// 13T, or 14T. This changes the drive speed of the module (a pinion gear with
		// more teeth will result in a robot that drives faster).
		public static final int kDrivingMotorPinionTeeth = 14;

		// Calculations required for driving motor conversion factors and feed forward
		public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
		public static final double kWheelDiameterMeters = 0.0762;
		public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
		// 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
		// teeth on the bevel pinion
		public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
		public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
				/ kDrivingMotorReduction;
	}

	public static final class OIConstants {
		public static final int kDriverControllerPort = 0;
		public static final double kDriveDeadband = 0.05;
	}

	public static final class AutoConstants {
		public static final double kMaxSpeedMetersPerSecond = 3;
		public static final double kMaxAccelerationMetersPerSecondSquared = 3;
		public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
		public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

		public static final double kPTransController = 4; // 4 is ok but not good
		// public static final double kPYController = 1;
		public static final double kPThetaController = 0.00625;

		// Constraint for the motion profiled robot angle controller
		public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
				kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
	}

	public static final class NeoMotorConstants {
		public static final double kFreeSpeedRpm = 5676;
	}

	public static final class CameraConstants {
		public static final Transform3d kRobotToCam = new Transform3d(
				new Translation3d(Units.inchesToMeters(6.5), 0.0, Units.inchesToMeters(24.5)),
				new Rotation3d(0, 0, 0));
		public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.5, 0.5, 1); // old: 4, 4, 8
		public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
	}

	public static final class FieldConstants {

		// FIELD POSITION CONSTANTS
		public static final Pose2d kRedHubPose = new Pose2d(Units.inchesToMeters(469.44), Units.inchesToMeters(158.84),
				new Rotation2d(0));
		public static final Pose2d kRedTrenchLeftPose = new Pose2d(Units.inchesToMeters(470.59),
				Units.inchesToMeters(25.37), new Rotation2d(0));

		public static final AprilTagFieldLayout kTagLayoutComp = AprilTagFieldLayout
				.loadField(AprilTagFields.kDefaultField);

		public static AprilTagFieldLayout LoadLayout(boolean isHome) {
			if (isHome) {
				try {
					// System.out.println(Filesystem.getDeployDirectory().getPath());
					// System.out.println(new File(Filesystem.getDeployDirectory(),
					// "fields/april_tag_layouts/home.json").getPath());
					return new AprilTagFieldLayout(
							new File(Filesystem.getDeployDirectory(), "fields/april_tag_layouts/home.json").getPath());
				} catch (IOException e) {
					System.err.println("Could not load custom home field layout");
					System.err.println(e);
					return kTagLayoutComp;
				}
			} else {
				try {
					// System.out.println(Filesystem.getDeployDirectory().getPath());
					// System.out.println(new File(Filesystem.getDeployDirectory(),
					// "fields/april_tag_layouts/home.json").getPath());
					return new AprilTagFieldLayout(
							new File(Filesystem.getDeployDirectory(), "fields/april_tag_layouts/2026welded.json")
									.getPath());
				} catch (IOException e) {
					System.err.println("Could not load custom home field layout");
					System.err.println(e);
					return kTagLayoutComp;
				}
			}

		}
	}

	public static final class PathPlannerConstants {
		public static final PathConstraints kPathFollowingConstraints = new PathConstraints(
				0.05, 0.01,
				Units.degreesToRadians(180), Units.degreesToRadians(360));
	}
}