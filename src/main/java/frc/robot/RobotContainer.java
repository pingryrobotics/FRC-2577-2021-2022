/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
// import commands and subsystems
// import frc.robot.commands.ChangeShooterDirection;
// import frc.robot.commands.ChangeShooterSpeed;
// import frc.robot.commands.ToggleHopper;
// import frc.robot.commands.ToggleIntake;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.Hopper;
// import frc.robot.subsystems.Intake;
// import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
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
	public Joystick m_driverController1 = new Joystick(0);
	public Joystick m_driverController2 = new Joystick(1);
	public Joystick m_driverController3 = new Joystick(2);

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
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An ExampleCommand will run in autonomous
		return null;
	}
}