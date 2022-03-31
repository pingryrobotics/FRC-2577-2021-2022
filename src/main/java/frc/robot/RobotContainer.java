/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.sql.Driver;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
// import commands and subsystems
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.commands.autos.OneBallAuto;
import frc.robot.commands.autos.TwoCargoAuto;
import frc.robot.commands.climb_commands.ExtendableClimb;
import frc.robot.commands.climb_commands.ReverseExtendableClimb;
import frc.robot.commands.climb_commands.ReverseRotatingClimb;
import frc.robot.commands.climb_commands.RotatingClimb;
import frc.robot.commands.drive_commands.ChangeDriveSpeed;
import frc.robot.commands.drive_commands.SetDriveDirection;
import frc.robot.commands.drive_commands.SetDriveSpeed;
import frc.robot.commands.intake_commands.IntakeBelt;
import frc.robot.commands.intake_commands.IntakeLiftDown;
import frc.robot.commands.intake_commands.IntakeLiftUp;
import frc.robot.commands.intake_commands.ReverseIntake;
import frc.robot.commands.intake_commands.ReverseIntakeAndBelt;
import frc.robot.commands.intake_commands.ReverseIntakeBelt;
import frc.robot.commands.intake_commands.SetBeltDirection;
import frc.robot.commands.intake_commands.SetBeltEnabled;
import frc.robot.commands.intake_commands.SetIntakeDirection;
import frc.robot.commands.intake_commands.SetIntakeEnabled;
import frc.robot.commands.intake_commands.ToggleBelt;
import frc.robot.commands.intake_commands.ToggleIntake;
import frc.robot.commands.intake_commands.ToggleIntakeAndBelt;
import frc.robot.commands.shooter_commands.ChangeShooterSpeed;
import frc.robot.commands.shooter_commands.IndexerIn;
import frc.robot.commands.shooter_commands.IndexerOut;
//import frc.robot.commands.TwoCargoAuto;
import frc.robot.subsystems.Climber;
// import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeLift;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.util.GeomUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
	// public XboxController m_driverController = new XboxController(0);
	public Joystick m_leftStick = new Joystick(0);
	public Joystick m_rightStick = new Joystick(1);
	public Joystick m_oliviaMechanism = new Joystick(2);
	public XboxController m_mechanismController = new XboxController(3);
	public XboxController driveController = new XboxController(4);
	// private final CANSparkMax leftMotor1 = new CANSparkMax(Constants.kLeftMotor1Port, MotorType.kBrushless);
	// private final CANSparkMax leftMotor2 = new CANSparkMax(Constants.kLeftMotor2Port, MotorType.kBrushless);
	// private final CANSparkMax rightMotor1 = new CANSparkMax(Constants.kRightMotor1Port, MotorType.kBrushless);
	// private final CANSparkMax rightMotor2 = new CANSparkMax(Constants.kRightMotor2Port, MotorType.kBrushless);
	// private final CANSparkMax leftMotor1 = new CANSparkMax(Constants.kLeftMotor1Port, MotorType.kBrushless);
	// private final CANSparkMax leftMotor2 = new CANSparkMax(Constants.kLeftMotor2Port, MotorType.kBrushless);
	// private final CANSparkMax rightMotor1 = new CANSparkMax(Constants.kRightMotor1Port, MotorType.kBrushless);
	// private final CANSparkMax rightMotor2 = new CANSparkMax(Constants.kRightMotor2Port, MotorType.kBrushless);
	// private MotorControllerGroup m_leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2);

	// The motors on the right side of the drive.
	// private MotorControllerGroup m_rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2);
	// private DifferentialDrive m_robotDiffDrive;
	private Drive m_robotDrive = new Drive();

	// private DifferentialSubsystem m_diffSub;
	// private final Drive m_robotDrive = new Drive();
	private final Shooter m_shooter = new Shooter(new CANSparkMax(Constants.kOuttakeLId, MotorType.kBrushless), new CANSparkMax(Constants.kOuttakeRId, MotorType.kBrushless));
	private final Intake m_intake = new Intake(new CANSparkMax(Constants.kIntakeId, MotorType.kBrushless), new CANSparkMax(Constants.kBeltId, MotorType.kBrushless));
	private final IntakeLift m_intakeLift = new IntakeLift(new CANSparkMax(Constants.kIntakeArmId, MotorType.kBrushless));
	// private final Climber m_climber = new Climber(new CANSparkMax(Constants.kClimberId, MotorType.kBrushless), new CANSparkMax(Constants.kRotatingArmId, MotorType.kBrushless));
	private final Climber m_climber = new Climber(new CANSparkMax(Constants.kClimberId, MotorType.kBrushless),
			new CANSparkMax(Constants.kRotatingArmId, MotorType.kBrushless));
	private final Indexer m_indexer = new Indexer(new CANSparkMax(Constants.kIndexerId, MotorType.kBrushless));
	private final Vision m_vision = new Vision(m_shooter);
	 // private final ColorSensor m_colorSensor = new ColorSensor(m_intake);
	// private Drive m_robotSubsystemDrive;
	private double speed = 1;
	private Command m_autonomousCommand;
	private boolean isForwards = true;
	public boolean intakeReversed = false;
	public boolean intakeOn = false;
	private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight"); 
	private double kP = 0.05;
	private double minValue = 0.05;


	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the button binding
		configureButtonBindings();

		CameraServer.startAutomaticCapture();
		// USE TO GET OUTPUT STUFF FROM CAMERA
		// Creates the CvSink and connects it to the UsbCamera
		// CvSink cvSink = CameraServer.getVideo();

		// Creates the CvSource and MjpegServer [2] and connects them
		// CvSource outputStream = CameraServer.putVideo("Blur", 640, 480);


		// m_robotDiffDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);
		// m_leftMotors.setInverted(true);
		// m_rightMotors.setInverted(true);
		// m_diffSub = new DifferentialSubsystem(m_leftMotors, m_rightMotors, m_robotDiffDrive);

		// m_intake.toggleBeltStart();
		
		// m_robotSubsystemDrive = new Drive(m_leftMotors, m_rightMotors);
		m_chooser.setDefaultOption("TA", new TwoCargoAuto(AutoPosition.TARMAC_A, m_robotDrive, m_intake, m_shooter));
		// m_chooser.addOption("TB", new TwoCargoAuto(AutoPosition.TARMAC_B, m_robotDrive, m_intake, m_shooter));
		m_chooser.addOption("TC", new TwoCargoAuto(AutoPosition.TARMAC_C, m_robotDrive, m_intake, m_shooter));
		m_chooser.addOption("TD", new TwoCargoAuto(AutoPosition.TARMAC_D, m_robotDrive, m_intake, m_shooter));
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
		new JoystickButton(m_mechanismController, Button.kX.value).whenPressed(new ChangeShooterSpeed(m_shooter, 1.0));
		new JoystickButton(m_mechanismController, Button.kA.value).whenPressed(new ChangeShooterSpeed(m_shooter, 0));
		new JoystickButton(m_mechanismController, Button.kStart.value).whenPressed(new ChangeShooterSpeed(m_shooter, -0.4));
		// new JoystickButton(m_mechanismController, Button.kA.value).whenPressed(new ChangeShooterSpeed(m_shooter, 0.3));
		new JoystickButton(m_mechanismController, Button.kB.value).whenPressed(new ChangeShooterSpeed(m_shooter, 0.60)); // lower level
		new JoystickButton(m_mechanismController, Button.kY.value).whenPressed(new ChangeShooterSpeed(m_shooter, 1)); // upper level
		// new JoystickButton(m_driverController1, 11).whenPressed(new ToggleHopper(m_hopper));
		new JoystickButton(m_mechanismController, Button.kRightBumper.value).whenPressed(new ReverseIntakeAndBelt(m_intake)); // toggle intake
		new JoystickButton(m_mechanismController, Button.kLeftBumper.value).whenPressed(new ToggleIntakeAndBelt(m_intake)); // toggle intake belt
		new POVButton(m_mechanismController, 180).whenHeld(new IntakeLiftUp(m_intakeLift)); // extend/retract arm
		new POVButton(m_mechanismController, 0).whenHeld(new IntakeLiftDown(m_intakeLift));
		new POVButton(m_mechanismController, 90).whenHeld(new RotatingClimb(m_climber, 0.1)); // rotate arm
		new POVButton(m_mechanismController, 270).whenHeld(new ReverseRotatingClimb(m_climber, 0.1));
		// new JoystickButton(m_mechanismController, Button.kBack.value).whenPressed(new ToggleColorSensor(m_colorSensor));

		SmartDashboard.putString("Direction", intakeReversed ? "Going out" : "Going in");
		SmartDashboard.putString("Intake status", intakeOn ? "On" : "Off");


		// new JoystickButton(m_mechanismController, Button.kBack.value).whenPressed(new ReverseIntake(m_intake)); // reverse intake
		// new JoystickButton(m_0mechanismController, Button.kStart.value).whenPressed(new ReverseIntakeBelt(m_intake)); // reverse

		// new JoystickButton(m_driverController, Button.kLeftBumper.value).whenHeld(new Climb(m_climber)); // toggle climber
		// new JoystickButton(m_driverController, Button.kRightBumper.value).whenHeld(new ReverseClimb(m_climber)); // toggle climber

		// olivia mechanism controller
		// climb
		new JoystickButton(m_oliviaMechanism, 6).whenHeld(new RotatingClimb(m_climber, 1));
		new JoystickButton(m_oliviaMechanism, 7).whenHeld(new ReverseRotatingClimb(m_climber, 1));
		new JoystickButton(m_oliviaMechanism, 10).whenHeld(new ExtendableClimb(m_climber));
		new JoystickButton(m_oliviaMechanism, 11).whenHeld(new ReverseExtendableClimb(m_climber));

		// intake

		// belt and indexer on while held
		new JoystickButton(m_oliviaMechanism, 1).whenPressed(new SequentialCommandGroup(
			new SetBeltDirection(m_intake, true),	
			new SetBeltEnabled(m_intake, true),
			new IndexerOut(m_indexer)));
		// belt off when released
		new JoystickButton(m_oliviaMechanism, 1).whenReleased(new SetBeltEnabled(m_intake, false));

		// intake on and intaking while held
		new JoystickButton(m_oliviaMechanism, 8).whenPressed(new SequentialCommandGroup(
			new SetIntakeDirection(m_intake, true),	
			new SetIntakeEnabled(m_intake, true),
			new SetBeltDirection(m_intake, true),
			new SetBeltEnabled(m_intake, true)));
		
		// intake off when released
		new JoystickButton(m_oliviaMechanism, 8).whenReleased(new SequentialCommandGroup(
			new SetIntakeEnabled(m_intake, false),
			new SetBeltEnabled(m_intake, false)));

		// intake on and reversed while held
		new JoystickButton(m_oliviaMechanism, 9).whenPressed(new SequentialCommandGroup(
			new SetIntakeDirection(m_intake, false),	
			new SetIntakeEnabled(m_intake, true),
			new SetBeltDirection(m_intake, false),
			new SetBeltEnabled(m_intake, true)));
		// intake off when released
		new JoystickButton(m_oliviaMechanism, 9).whenReleased(new SequentialCommandGroup(
			new SetIntakeEnabled(m_intake, false),
			new SetBeltEnabled(m_intake, false)));

		new JoystickButton(m_oliviaMechanism, 4).whenPressed(new IntakeLiftUp(m_intakeLift));
		new JoystickButton(m_oliviaMechanism, 5).whenPressed(new IntakeLiftDown(m_intakeLift));
		new JoystickButton(m_oliviaMechanism, 3).whenPressed(new ChangeShooterSpeed(m_shooter, .7));
		new JoystickButton(m_oliviaMechanism, 3).whenReleased(new ChangeShooterSpeed(m_shooter, 0));



		

		new JoystickButton(m_leftStick, 3).whenPressed(new IndexerOut(m_indexer));
		new JoystickButton(m_leftStick, 2).whenPressed(new IndexerIn(m_indexer));

		new JoystickButton(m_rightStick, 3).whenHeld(new ExtendableClimb(m_climber));
		new JoystickButton(m_rightStick, 2).whenHeld(new ReverseExtendableClimb(m_climber));
		// new JoystickButton(m_rightStick, 4).whenHeld(new SetDriveDirection(m_diffSub, true));
		// new JoystickButton(m_rightStick, 5).whenHeld(new SetDriveDirection(m_diffSub, false));




		// new JoystickButton(m_leftStick, 4).whenPressed(new ChangeDriveSpeed(m_robotSubsystemDrive));
		// new JoystickButton(m_leftStick, 5).whenPressed(new ChangeDriveDirection(m_robotSubsystemDrive));
		// m_mechanismController.getTriggerAxis(Hand.kLeft).whenActive(new ReverseIntakeBelt(m_intake));
		// m_mechanismController.getTriggerAxis(Hand.kRight).whenActive(new ReverseIntake(m_intake));

		// new JoystickButton(m_leftStick, 1).whenPressed(new ToggleIntake(m_robotDrive));
		// new JoyStickButton(m_mechanismController, 1).whenPressed(new (m_robotDrive));
	}

	public void driveControl() {
		// m_robotDrive.tankDrive(m_leftStick.getY(), m_rightStick.getY());
		SmartDashboard.putNumber("Current Heading", m_robotDrive.getHeading());
		SmartDashboard.putNumber("Current Simple Heading", m_robotDrive.getSimpleAngle());
		SmartDashboard.putNumber("Turn rate", m_robotDrive.getTurnRate());


		if(driveController.getLeftBumper()){
			if(table.getEntry("tv").getDouble(0.0) == 1){
				double tx = table.getEntry("tx").getDouble(0.0);
				double headingError = -table.getEntry("tx").getDouble(0.0);
				double steeringAdjust = 0.0;
				if(tx > 0){
					steeringAdjust = kP*headingError - minValue;
				}
				else if(tx < 1.0){
					steeringAdjust = kP*headingError + minValue;
				}
				m_robotDrive.tankDrive(steeringAdjust, -steeringAdjust);
			}
		}
		
		else{
			if (Math.abs(driveController.getLeftY()) < .2) {
				m_robotDrive.curvatureDrive(driveController.getLeftY(), driveController.getRightX(), true);
			} else {
				m_robotDrive.curvatureDrive(driveController.getLeftY(), driveController.getRightX(), driveController.getRightBumper());
			}
		}

		
		// m_robotSubsystemDrive.tankDrive(m_leftStick.getY(), m_rightStick.getY());
		

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




		m_robotDrive.setPose(m_chooser.getSelected().pos.getPose());
		System.out.println("chosen auto pos: " + m_chooser.getSelected().pos.getPose());
		return m_chooser.getSelected();
		// return void;
	}


	public void oneBall(){
		
		// m_robotDiffDrive.stopMotor();
		
		// // m_autonomousCommand = getAutonomousCommand();
		// // schedule the autonomous command (example)
		// // if (m_autonomousCommand != null) {
		// 	// m_autonomousCommand.schedule();
		// // }
		// Command command = new OneBallAuto(m_intake, m_shooter);
		// if (command != null) {
		// 	command.schedule();
		// }
		
		// m_robotDiffDrive.tankDrive(-0.5, -0.5);
		// new WaitCommand(1).schedule();
		// m_robotDiffDrive.stopMotor();
	}

	// public void twoBalls(){
	// 	ADIS16470_IMU m_imu = new ADIS16470_IMU(); 
	// 	double yawOriginal = m_imu.getAngle();
	// 	// new ToggleIntake(m_intake).schedule();
	// 	new ToggleIntakeAndBelt(m_intake).schedule();
	// 	m_robotDiffDrive.tankDrive(0.5, 0.5);
	// 	new WaitCommand(1).schedule();
	// 	m_robotDiffDrive.stopMotor();
	// 	new ToggleIntakeAndBelt(m_intake).end(true);
	// 	m_robotDiffDrive.tankDrive(0.3, -0.3);
	// 	while(180 >= Math.abs(yawOriginal-m_imu.getAngle())){
	// 		new WaitCommand(0.05).schedule();
	// 	}
	// 	m_robotDiffDrive.stopMotor();

	// 	m_robotDiffDrive.tankDrive(0.5, 0.5);
	// 	new WaitCommand(1).schedule();
	// 	m_robotDiffDrive.stopMotor();
		
	// 	new ReverseIntakeBelt(m_intake).schedule();
	// 	new WaitCommand(0.5).schedule();
	// 	new ReverseIntakeBelt(m_intake).end(true);
	// 	new ChangeShooterSpeed(m_shooter, 0.7).schedule();
	// 	new WaitCommand(1).schedule();
	// 	new ToggleIntakeAndBelt(m_intake).schedule();
	// 	new WaitCommand(3).schedule();

			
	// 	m_robotDiffDrive.tankDrive(-0.5, -0.5);
	// 	new WaitCommand(1).schedule();
	// 	m_robotDiffDrive.stopMotor();
	// }
		// Map<String, AutoRoutine> autoMap = new HashMap<String, AutoRoutine>();
		// autoMap.put("Two cargo (TA)",
        // 	new AutoRoutine(AutoPosition.TARMAC_A,
        // 	    new TwoCargoAuto(AutoPosition.TARMAC_A, m_robotDrive, m_intake, m_shooter)));
		// autoMap.put("Two cargo (TB)",
		// 	new AutoRoutine(AutoPosition.TARMAC_B,
		// 		new TwoCargoAuto(AutoPositioPn.TARMAC_B, m_robotDrive, m_intake, m_shooter)));
		// autoMap.put("Two cargo (TC)",
		// 	new AutoRoutine(AutoPosition.TARMAC_C,
		// 		new TwoCargoAuto(AutoPosition.TARMAC_C, m_robotDrive, m_intake, m_shooter)));
		// autoMap.put("Two cargo (TD)",
		// 	new AutoRoutine(AutoPosition.TARMAC_D,
		// 				new TwoCargoAuto(AutoPosition.TARMAC_D, m_robotDrive, m_intake, m_shooter)));




		// m_robotDrive.setPose(m_chooser.getSelected().pos.getPose());
		//return m_chooser.getSelected();



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
	// }

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