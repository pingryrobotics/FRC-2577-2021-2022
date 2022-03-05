/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.util.GeomUtil;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be
 * declared globally (i.e. public static). Do not put anything functional in
 * this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	// PATH PLANNING CONSTANTS -- WILL NEED TO BE CHANGED FOR EACH ROBOT
	public static final double kTrackwidthMeters = 0.69;
	public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
			kTrackwidthMeters);
	public static final double kMaxSpeedMetersPerSecond = 3;
	public static final double kMaxAccelerationMetersPerSecondSquared = 3;
	// Reasonable baseline values for a RAMSETE follower in units of meters and
	// seconds
	public static final double kRamseteB = 2;
	public static final double kRamseteZeta = 0.7;

	// https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/characterizing-drive.html
	// These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
	// These characterization values MUST be determined either experimentally or
	// theoretically
	// for *your* robot's drive.
	// The Robot Characterization Toolsuite provides a convenient tool for obtaining
	// these
	// values for your robot.
	public static final double ksVolts = 0.22;
	public static final double kvVoltSecondsPerMeter = 1.98;
	public static final double kaVoltSecondsSquaredPerMeter = 0.2;
	// Example value only - as above, this must be tuned for your drive!
	public static final double kPDriveVel = 8.5;
	public static final int kLeftMotor1Port = 10;
	public static final int kLeftMotor2Port = 11;
	public static final int kRightMotor1Port = 12;
	public static final int kRightMotor2Port = 13;
	public static final boolean kLeftEncoderReversed = false;
	public static final boolean kRightEncoderReversed = false;
	// public static final int kEncoderDistancePerPulse = 10000; // change
	public static final double kDistancePerWheelRevolutionMeters = 0.478778; // pi * 6 inches to meters
	public static final double kDriveTrainGearReduction = 12;
	// public static final SPI.Port imuPort = SPI.Port.kOnboardCS0; // change
	public static final double kImageCaptureLatency = 11.0 / 1000.0;


	public static final int kOuttakeSpeed = 1;
	public static final int kIntakeSpeed = 1;
	public static final double kBeltSpeed = 0.5;
	public static final double kClimberSpeed = 0.5;
	public static final double kRotatingSpeed = 0.1;
	public static final int kSlideLimit = 1; // in rotations
	public static final int kArmLimit = 1; // in rotations
	public static final int kIntakeId = 20;
	public static final int kOuttakeLId = 22;
	public static final int kOuttakeRId = 23;
	public static final int kClimberId = 30;
	public static final int kRotatingArmId = 31;
	public static final int kBeltId = 21;


	// Field dimensions
	public static final double fieldLength = Units.inchesToMeters(54.0 * 12.0);
	public static final double fieldWidth = Units.inchesToMeters(27.0 * 12.0);
	public static final double hangarLength = Units.inchesToMeters(128.75);
	public static final double hangarWidth = Units.inchesToMeters(116.0);

	// Vision target
	public static final double visionTargetDiameter = Units.inchesToMeters(4.0 * 12.0 + 5.375);
	public static final double visionTargetHeightLower = Units.inchesToMeters(8.0 * 12 + 5.625); // Bottom of tape
	public static final double visionTargetHeightUpper = visionTargetHeightLower + Units.inchesToMeters(2.0); // Top of
																												// tape

	// Dimensions of hub and tarmac
	public static final Rotation2d centerLineAngle = Rotation2d.fromDegrees(66.0);
	public static final Translation2d hubCenter = new Translation2d(fieldLength / 2.0, fieldWidth / 2.0);
	public static final double tarmacInnerDiameter = Units.inchesToMeters(219.25);
	public static final double tarmacOuterDiameter = Units.inchesToMeters(237.31);
	public static final double tarmacFenderToTip = Units.inchesToMeters(84.75);
	public static final double tarmacFullSideLength = tarmacInnerDiameter * (Math.sqrt(2.0) - 1.0); // If the tarmac
																									// formed a full
																									// octagon
	public static final double tarmacMarkedSideLength = Units.inchesToMeters(82.83); // Length of tape marking outside
																						// of tarmac
	public static final double tarmacMissingSideLength = tarmacFullSideLength - tarmacMarkedSideLength; // Length
																										// removed b/c
																										// of corner
																										// cutoff
	public static final double hubSquareLength = tarmacOuterDiameter - (tarmacFenderToTip * 2.0);

	// Reference rotations (angle from hub to each reference point and fender side)
	public static final Rotation2d referenceARotation = Rotation2d.fromDegrees(180.0).minus(centerLineAngle)
			.plus(Rotation2d.fromDegrees(360.0 / 16.0));
	public static final Rotation2d referenceBRotation = referenceARotation
			.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0));
	public static final Rotation2d referenceCRotation = referenceBRotation
			.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0));
	public static final Rotation2d referenceDRotation = referenceCRotation
			.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0));
	public static final Rotation2d fenderARotation = referenceARotation.rotateBy(Rotation2d.fromDegrees(360.0 / 16.0));
	public static final Rotation2d fenderBRotation = fenderARotation.rotateBy(Rotation2d.fromDegrees(90.0));

	// Reference points (centered of the sides of the tarmac if they formed a
	// complete octagon, plus
	// edges of fender)
	public static final Pose2d referenceA = new Pose2d(hubCenter, referenceARotation).transformBy(
			GeomUtil.transformFromTranslation(tarmacInnerDiameter / 2.0, 0.0));
	public static final Pose2d referenceB = new Pose2d(hubCenter, referenceBRotation).transformBy(
			GeomUtil.transformFromTranslation(tarmacInnerDiameter / 2.0, 0.0));
	public static final Pose2d referenceC = new Pose2d(hubCenter, referenceCRotation).transformBy(
			GeomUtil.transformFromTranslation(tarmacInnerDiameter / 2.0, 0.0));
	public static final Pose2d referenceD = new Pose2d(hubCenter, referenceDRotation).transformBy(
			GeomUtil.transformFromTranslation(tarmacInnerDiameter / 2.0, 0.0));
	public static final Pose2d fenderA = new Pose2d(hubCenter, fenderARotation).transformBy(
			GeomUtil.transformFromTranslation(hubSquareLength / 2.0, 0.0));
	public static final Pose2d fenderB = new Pose2d(hubCenter, fenderBRotation).transformBy(
			GeomUtil.transformFromTranslation(hubSquareLength / 2.0, 0.0));

	// cargo points
	public static final double cornerToCargoY = Units.inchesToMeters(15.56);
	public static final double referenceToCargoY = (tarmacFullSideLength / 2.0) - cornerToCargoY;
	public static final double referenceToCargoX = Units.inchesToMeters(40.44);
	public static final Pose2d cargoA = referenceA.transformBy(
			GeomUtil.transformFromTranslation(referenceToCargoX, -referenceToCargoY));
	public static final Pose2d cargoB = referenceA.transformBy(
			GeomUtil.transformFromTranslation(referenceToCargoX, referenceToCargoY));
	public static final Pose2d cargoC = referenceB.transformBy(
			GeomUtil.transformFromTranslation(referenceToCargoX, referenceToCargoY));
	public static final Pose2d cargoD = referenceC.transformBy(
			GeomUtil.transformFromTranslation(referenceToCargoX, -referenceToCargoY));
	public static final Pose2d cargoE = referenceD.transformBy(
			GeomUtil.transformFromTranslation(referenceToCargoX, -referenceToCargoY));
	public static final Pose2d cargoF = referenceD.transformBy(
			GeomUtil.transformFromTranslation(referenceToCargoX, referenceToCargoY));
}