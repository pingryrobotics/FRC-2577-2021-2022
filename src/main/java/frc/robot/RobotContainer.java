/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
// import commands and subsystems

import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * // * "declarative" paradigm, very little robot logic should actually be
 * handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	// private final Shooter m_shooter = new Shooter(new CANSparkMax(Constants.kLeftShooterId, MotorType.kBrushless),
	// 		new CANSparkMax(Constants.kRightShooterId, MotorType.kBrushless),
	// 		new CANSparkMax(Constants.kLiftId, MotorType.kBrushless));
	// private final Hopper m_hopper = new Hopper(new CANSparkMax(Constants.kHopperLowerId, MotorType.kBrushless),
	// 		new CANSparkMax(Constants.kHopperUpperId, MotorType.kBrushless));
	// private final Intake m_intake = new Intake(new CANSparkMax(Constants.kIntakeId, MotorType.kBrushless));
	// private final DriveBase m_driveBase = new DriveBase();
	public Joystick m_leftStick = new Joystick(0);
	public Joystick m_rightStick = new Joystick(1);
	public Joystick m_mechanismController = new Joystick(2);
	private Drive m_robotDrive = new Drive();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the button bindings
		configureButtonBindings();
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link GenericHID} or one of its subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
	 * it to a
	 * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		// input commands here
		// new JoystickButton(m_driverController1, 1).whenPressed(new ChangeShooterDirection(m_shooter));
		// new JoystickButton(m_driverController1, 2).whenPressed(new ChangeShooterSpeed(m_shooter, 0));
		// new JoystickButton(m_driverController1, 3).whenPressed(new ChangeShooterSpeed(m_shooter, .5));
		// new JoystickButton(m_driverController1, 4).whenPressed(new ChangeShooterSpeed(m_shooter, .7));
		// new JoystickButton(m_driverController1, 5).whenPressed(new ChangeShooterSpeed(m_shooter, .8));
		// new JoystickButton(m_driverController1, 11).whenPressed(new ToggleHopper(m_hopper));
		// new JoystickButton(m_driverController1, 10).whenPressed(new ToggleIntake(m_intake));
		// new JoystickButton(m_leftStick, 1).whenPressed(new ToggleIntake(m_robotDrive));
		// new JoyStickButton(m_mechanismController, 1).whenPressed(new (m_robotDrive));
	}

	public void driveControl() {
		m_robotDrive.tankDrive(m_leftStick.getY(), m_rightStick.getY());
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// Create a voltage constraint to ensure we don't accelerate too fast
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

		// An example trajectory to follow. All units in meters.
		Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
				// Start at the origin facing the +X direction
				new Pose2d(0, 0, new Rotation2d(0)),
				// Pass through these two interior waypoints, making an 's' curve path
				List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
				// End 3 meters straight ahead of where we started, facing forward
				new Pose2d(3, 0, new Rotation2d(0)),
				// Pass config
				config);

		RamseteCommand ramseteCommand = new RamseteCommand(
				exampleTrajectory,
				m_robotDrive::getPose,
				new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
				new SimpleMotorFeedforward(
						Constants.ksVolts,
						Constants.kvVoltSecondsPerMeter,
						Constants.kaVoltSecondsSquaredPerMeter),
				Constants.kDriveKinematics,
				m_robotDrive::getWheelSpeeds,
				new PIDController(Constants.kPDriveVel, 0, 0),
				new PIDController(Constants.kPDriveVel, 0, 0),
				// RamseteCommand passes volts to the callback
				m_robotDrive::tankDriveVolts,
				m_robotDrive);

		// Reset odometry to the starting pose of the trajectory.
		m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

		// Run path following command, then stop at the end.
		return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
	}
}