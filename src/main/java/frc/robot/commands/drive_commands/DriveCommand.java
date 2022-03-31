// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive_commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class DriveCommand extends CommandBase {
	private static final double ramseteB = 2;
	private static final double ramseteZeta = 0.7;

	private final Drive drive;
	private final DifferentialDriveKinematics kinematics;
	private final Trajectory trajectory;
	private final RamseteController controller = new RamseteController(ramseteB, ramseteZeta);
	private final Timer timer = new Timer();

	/**
	 * Creates a new DriveCommand with no extra constraints. Drives along
	 * the specified path
	 * based on odometry data.
	 */
	public DriveCommand(Drive drive, double startVelocityMetersPerSec,
			List<Pose2d> waypoints, double endVelocityMetersPerSec,
			boolean reversed) {
		this(drive, startVelocityMetersPerSec, waypoints, endVelocityMetersPerSec,
				reversed, List.of());
	}

	/**
	 * Creates a new DriveCommand with extra constraints. Drives along the
	 * specified path
	 * based on odometry data.
	 */
	public DriveCommand(Drive drive, double startVelocityMetersPerSec,
			List<Pose2d> waypoints, double endVelocityMetersPerSec, boolean reversed,
			List<TrajectoryConstraint> constraints) {
		addRequirements(drive);
		this.drive = drive;
		kinematics = new DifferentialDriveKinematics(1.0);

		var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
				new SimpleMotorFeedforward(
						Constants.ksVolts,
						Constants.kvVoltSecondsPerMeter,
						Constants.kaVoltSecondsSquaredPerMeter),
				Constants.kDriveKinematics,
				10);
		// Create config for trajectory
		TrajectoryConfig config = new TrajectoryConfig(
				Constants.kMaxSpeedMetersPerSecond,
				Constants.kMaxAccelerationMetersPerSecondSquared)
						// Add kinematics to ensure max speed is actually obeyed
						.setKinematics(Constants.kDriveKinematics)
						// Apply the voltage constraint
						.addConstraint(autoVoltageConstraint);


		// limited to 2 points for now
		List<Translation2d> passPoints = new ArrayList<Translation2d>();
		// for (int waypointInd = 1; waypointInd < waypoints.size() - 2; waypointInd++) {
			// 
			// passPoints.add(waypoints.get(waypointInd));
		// }
		// An example trajectory to follow. All units in meters.
		Trajectory traj = TrajectoryGenerator.generateTrajectory(
				// Start at the origin facing the +X direction
				waypoints.get(0),
				// Pass through these two interior waypoints, making an 's' curve path
				passPoints,
				// End 3 meters straight ahead of where we started, facing forward
				waypoints.get(waypoints.size() - 1),
				// Pass config
				config);

		trajectory = traj;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		timer.reset();
		timer.start();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		State setpoint = trajectory.sample(timer.get());
		ChassisSpeeds chassisSpeeds = controller.calculate(drive.getPose(), setpoint);
		DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
		drive.tankDrive(wheelSpeeds.leftMetersPerSecond,
				wheelSpeeds.rightMetersPerSecond);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		timer.stop();
		drive.tankDrive(0, 0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return timer.hasElapsed(trajectory.getTotalTimeSeconds());
	}

	public double getDuration() {
		return trajectory.getTotalTimeSeconds();
	}
}