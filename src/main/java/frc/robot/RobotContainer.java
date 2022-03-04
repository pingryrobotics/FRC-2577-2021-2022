/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
// import commands and subsystems
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.Climb;
import frc.robot.commands.ReverseClimb;
// import frc.robot.commands.ChangeRotatingClimberSpeed;
import frc.robot.commands.ChangeShooterSpeed;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.ReverseIntakeBelt;
import frc.robot.commands.ReverseRotatingClimb;
import frc.robot.commands.RotatingClimb;
import frc.robot.commands.ToggleIntake;
import frc.robot.commands.ToggleIntakeBelt;
import frc.robot.commands.TwoCargoAuto;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.util.GeomUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

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
	public XboxController m_driverController = new XboxController(0);
	// public Joystick m_leftStick = new Joystick(0);
	// public Joystick m_rightStick = new Joystick(1);
	public XboxController m_mechanismController = new XboxController(1);
	// private final Drive m_robotDrive = new Drive();
	private final Shooter m_shooter = new Shooter(new CANSparkMax(Constants.kOuttakeLId, MotorType.kBrushless), new CANSparkMax(Constants.kOuttakeRId, MotorType.kBrushless));
	private final Intake m_intake = new Intake(new CANSparkMax(Constants.kIntakeId, MotorType.kBrushless), new CANSparkMax(Constants.kBeltId, MotorType.kBrushless));
	// private final Climber m_climber = new Climber(new CANSparkMax(Constants.kClimberId, MotorType.kBrushless), new CANSparkMax(Constants.kRotatingArmId, MotorType.kBrushless));
	private final Climber m_climber = new Climber(new CANSparkMax(Constants.kClimberId, MotorType.kBrushless), new CANSparkMax(Constants.kRotatingArmId, MotorType.kBrushless));


	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the button binding
		configureButtonBindings();
		// m_chooser.setDefaultOption("TA", new TwoCargoAuto(AutoPosition.TARMAC_A, m_robotDrive, m_intake, m_shooter));
		// m_chooser.addOption("TB", new TwoCargoAuto(AutoPosition.TARMAC_B, m_robotDrive, m_intake, m_shooter));
		// m_chooser.addOption("TC", new TwoCargoAuto(AutoPosition.TARMAC_C, m_robotDrive, m_intake, m_shooter));
		// m_chooser.addOption("TD", new TwoCargoAuto(AutoPosition.TARMAC_D, m_robotDrive, m_intake, m_shooter));
		SmartDashboard.putData("Auto choices", m_chooser);
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
		new JoystickButton(m_mechanismController, Button.kX.value).whenPressed(new ChangeShooterSpeed(m_shooter, -0.1));
		new JoystickButton(m_mechanismController, Button.kA.value).whenPressed(new ChangeShooterSpeed(m_shooter, 0));
		new JoystickButton(m_mechanismController, Button.kY.value).whenPressed(new ChangeShooterSpeed(m_shooter, .5)); // lower level
		new JoystickButton(m_mechanismController, Button.kB.value).whenPressed(new ChangeShooterSpeed(m_shooter, 1)); // upper level
		// new JoystickButton(m_driverController1, 11).whenPressed(new ToggleHopper(m_hopper));
		new JoystickButton(m_mechanismController, Button.kLeftBumper.value).whenPressed(new ToggleIntake(m_intake)); // toggle intake
		new JoystickButton(m_mechanismController, Button.kRightBumper.value).whenPressed(new ToggleIntakeBelt(m_intake)); // toggle intake belt
		new POVButton(m_mechanismController, 0).whenPressed(new Climb(m_climber)); // extend/retract arm
		new POVButton(m_mechanismController, 90).whenPressed(new ReverseClimb(m_climber));
		new POVButton(m_mechanismController, 180).whenPressed(new RotatingClimb(m_climber)); // rotate arm
		new POVButton(m_mechanismController, 270).whenPressed(new ReverseRotatingClimb(m_climber));

		new JoystickButton(m_mechanismController, Button.kLeftStick.value).whenPressed(new ReverseIntake(m_intake)); // reverse intake
		new JoystickButton(m_mechanismController, Button.kRightStick.value).whenPressed(new ReverseIntakeBelt(m_intake)); // reverse

		new JoystickButton(m_driverController, Button.kLeftBumper.value).whenPressed(new Climb(m_climber)); // toggle climber
		new JoystickButton(m_driverController, Button.kRightBumper.value).whenPressed(new ReverseClimb(m_climber)); // toggle climber
		// m_mechanismController.getTriggerAxis(Hand.kLeft).whenActive(new ReverseIntakeBelt(m_intake));
		// m_mechanismController.getTriggerAxis(Hand.kRight).whenActive(new ReverseIntake(m_intake));

		// new JoystickButton(m_leftStick, 1).whenPressed(new ToggleIntake(m_robotDrive));
		// new JoyStickButton(m_mechanismController, 1).whenPressed(new (m_robotDrive));
	}

	public void driveControl() {
		// m_robotDrive.tankDrive(m_leftStick.getY(), m_leftStick.getX());
		// m_robotDrive.tankDrive(m_driverController.getRawAxis(Axis.kLeftY.value), m_driverController.getRawAxis(Axis.kRightY.value));
	}

	// A chooser for autonomous commands
	SendableChooser<TwoCargoAuto> m_chooser = new SendableChooser<>();

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// Map<String, AutoRoutine> autoMap = new HashMap<String, AutoRoutine>();
		// autoMap.put("Two cargo (TA)",
        // 	new AutoRoutine(AutoPosition.TARMAC_A,
        // 	    new TwoCargoAuto(AutoPosition.TARMAC_A, m_robotDrive, m_intake, m_shooter)));
		// autoMap.put("Two cargo (TB)",
		// 	new AutoRoutine(AutoPosition.TARMAC_B,
		// 		new TwoCargoAuto(AutoPosition.TARMAC_B, m_robotDrive, m_intake, m_shooter)));
		// autoMap.put("Two cargo (TC)",
		// 	new AutoRoutine(AutoPosition.TARMAC_C,
		// 		new TwoCargoAuto(AutoPosition.TARMAC_C, m_robotDrive, m_intake, m_shooter)));
		// autoMap.put("Two cargo (TD)",
		// 	new AutoRoutine(AutoPosition.TARMAC_D,
		// 				new TwoCargoAuto(AutoPosition.TARMAC_D, m_robotDrive, m_intake, m_shooter)));




		// m_robotDrive.setPose(m_chooser.getSelected().pos.getPose());
		return m_chooser.getSelected();
		// // Create a voltage constraint to ensure we don't accelerate too fast
		// var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
		// 		new SimpleMotorFeedforward(
		// 				Constants.ksVolts,
		// 				Constants.kvVoltSecondsPerMeter,
		// 				Constants.kaVoltSecondsSquaredPerMeter),
		// 		Constants.kDriveKinematics,
		// 		10);

		// // Create config for trajectory
		// TrajectoryConfig config = new TrajectoryConfig(
		// 		Constants.kMaxSpeedMetersPerSecond,
		// 		Constants.kMaxAccelerationMetersPerSecondSquared)
		// 				// Add kinematics to ensure max speed is actually obeyed
		// 				.setKinematics(Constants.kDriveKinematics)
		// 				// Apply the voltage constraint
		// 				.addConstraint(autoVoltageConstraint);


		// // An example trajectory to follow. All units in meters.
		// Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
		// 		// Start at the origin facing the +X direction
		// 		new Pose2d(0, 0, new Rotation2d(0)),
		// 		// Pass through these two interior waypoints, making an 's' curve path
		// 		List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
		// 		// End 3 meters straight ahead of where we started, facing forward
		// 		new Pose2d(3, 0, new Rotation2d(0)),
		// 		// Pass config
		// 		config);

		// RamseteCommand ramseteCommand = new RamseteCommand(
		// 		exampleTrajectory,
		// 		m_robotDrive::getPose,
		// 		new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
		// 		new SimpleMotorFeedforward(
		// 				Constants.ksVolts,
		// 				Constants.kvVoltSecondsPerMeter,
		// 				Constants.kaVoltSecondsSquaredPerMeter),
		// 		Constants.kDriveKinematics,
		// 		m_robotDrive::getWheelSpeeds,
		// 		new PIDController(Constants.kPDriveVel, 0, 0),
		// 		new PIDController(Constants.kPDriveVel, 0, 0),
		// 		// RamseteCommand passes volts to the callback
		// 		m_robotDrive::tankDriveVolts,
		// 		m_robotDrive);

		// // Reset odometry to the starting pose of the trajectory.
		// m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

		// // Run path following command, then stop at the end.
		// return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
	}

	public static enum AutoPosition {
		ORIGIN, TARMAC_A, TARMAC_B, TARMAC_C, TARMAC_D, FENDER_A, FENDER_B;

		public Pose2d getPose() {
			switch (this) {
				case ORIGIN:
					return new Pose2d();
				case TARMAC_A:
					return Constants.referenceA
							.transformBy(GeomUtil.transformFromTranslation(-0.5, 0.7));
				case TARMAC_B:
					return Constants.referenceB
							.transformBy(GeomUtil.transformFromTranslation(-0.5, -0.2));
				case TARMAC_C:
					return Constants.referenceC
							.transformBy(GeomUtil.transformFromTranslation(-0.5, -0.1));
				case TARMAC_D:
					return Constants.referenceD
							.transformBy(GeomUtil.transformFromTranslation(-0.5, -0.7));
				case FENDER_A:
					return Constants.fenderA
							.transformBy(GeomUtil.transformFromTranslation(0.5, 0.0));
				case FENDER_B:
					return Constants.fenderB
							.transformBy(GeomUtil.transformFromTranslation(0.5, 0.0));
				default:
					return new Pose2d();
			}
		}
	}
}