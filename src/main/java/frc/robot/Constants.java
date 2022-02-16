/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.SPI;

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
	public static final int kIntakeSpeed = -1;

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
	public static final int kLeftMotor1Port = -1;
	public static final int kLeftMotor2Port = -1;
	public static final int kRightMotor1Port = -1;
	public static final int kRightMotor2Port = -1;
	public static final int[] kLeftEncoderPorts = { -1, -1 };
	public static final int[] kRightEncoderPorts = { -1, -1 };
	public static final boolean kLeftEncoderReversed = false;
	public static final boolean kRightEncoderReversed = false;
	public static final int kEncoderDistancePerPulse = -1;
	// public static final SPI.Port imuPort = SPI.Port.kOnboardCS0; // change
}