// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import java.util.List;

// import edu.wpi.first.math.controller.RamseteController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
// import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.math.trajectory.Trajectory.State;
// import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
// import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
// import frc.robot.Constants;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Drive;

// public class DriveCommand extends CommandBase {
// 	private static final double ramseteB = 2;
// 	private static final double ramseteZeta = 0.7;

// 	private final Drive drive;
// 	private final DifferentialDriveKinematics kinematics;
// 	private final Trajectory trajectory;
// 	private final RamseteController controller = new RamseteController(ramseteB, ramseteZeta);
// 	private final Timer timer = new Timer();

// 	/**
// 	 * Creates a new DriveCommand with no extra constraints. Drives along
// 	 * the specified path
// 	 * based on odometry data.
// 	 */
// 	public DriveCommand(Drive drive, double startVelocityMetersPerSec,
// 			List<Pose2d> waypoints, double endVelocityMetersPerSec,
// 			boolean reversed) {
// 		this(drive, startVelocityMetersPerSec, waypoints, endVelocityMetersPerSec,
// 				reversed, List.of());
// 	}

// 	/**
// 	 * Creates a new DriveCommand with extra constraints. Drives along the
// 	 * specified path
// 	 * based on odometry data.
// 	 */
// 	public DriveCommand(Drive drive, double startVelocityMetersPerSec,
// 			List<Pose2d> waypoints, double endVelocityMetersPerSec, boolean reversed,
// 			List<TrajectoryConstraint> constraints) {
// 		addRequirements(drive);
// 		this.drive = drive;
// 		kinematics = new DifferentialDriveKinematics(1.0);

// 		var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
// 				new SimpleMotorFeedforward(
// 						Constants.ksVolts,
// 						Constants.kvVoltSecondsPerMeter,
// 						Constants.kaVoltSecondsSquaredPerMeter),
// 				Constants.kDriveKinematics,
// 				10);
// 				h
// 		// Create config for trajectory
// 		TrajectoryConfig config = new TrajectoryConfig(
// 				Constants.kMaxSpeedMetersPerSecond,
// 				Constants.kMaxAccelerationMetersPerSecondSquared)
// 						// Add kinematics to ensure max speed is actually obeyed
// 						.setKinematics(Constants.kDriveKinematics)
// 						// Apply the voltage constraint
// 						.addConstraint(autoVoltageConstraint);


// 		// An example trajectory to follow. All units in meters.
// 		Trajectory traj = TrajectoryGenerator.generateTrajectory(
// 				// Start at the origin facing the +X direction
// 				new Pose2d(0, 0, new Rotation2d(0)),
// 				// Pass through these two interior waypoints, making an 's' curve path
// 				List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
// 				// End 3 meters straight ahead of where we started, facing forward
// 				new Pose2d(3, 0, new Rotation2d(0)),
// 				// Pass config
// 				config);

// 		trajectory = traj;
// 	}

// 	// Called when the command is initially scheduled.
// 	@Override
// 	public void initialize() {
// 		timer.reset();
// 		timer.start();
// 	}

// 	// Called every time the scheduler runs while the command is scheduled.
// 	@Override
// 	public void execute() {
// 		State setpoint = trajectory.sample(timer.get());
// 		ChassisSpeeds chassisSpeeds = controller.calculate(drive.getPose(), setpoint);
// 		DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
// 		drive.tankDrive(wheelSpeeds.leftMetersPerSecond,
// 				wheelSpeeds.rightMetersPerSecond);
// 	}

// 	// Called once the command ends or is interrupted.
// 	@Override
// 	public void end(boolean interrupted) {
// 		timer.stop();
// 		drive.tankDrive(0, 0);
// 	}

// 	// Returns true when the command should end.
// 	@Override
// 	public boolean isFinished() {
// 		return timer.hasElapsed(trajectory.getTotalTimeSeconds());
// 	}

// 	public double getDuration() {
// 		return trajectory.getTotalTimeSeconds();
// 	}
// }