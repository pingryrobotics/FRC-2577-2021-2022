// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.robot.commands;

// import frc.robot.subsystems.Drive;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.Subsystem;

// /**
//  * An example command that uses an example subsystem.
//  */
// public class ChangeDriveDirection extends CommandBase {
// 	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
// 	private final Drive m_subsystem;
// 	private double speed;

// 	/**
// 	 * Creates a new ExampleCommand.
// 	 *
// 	 * @param subsystem The subsystem used by this command.
// 	 */
// 	public ChangeDriveDirection(Drive subsystem) {
// 		m_subsystem = subsystem;
// 		// Use addRequirements() here to declare subsystem dependencies.
// 		addRequirements(subsystem);

// 	}

// 	// Called when the command is initially scheduled.
// 	@Override
// 	public void initialize() {
// 		if(m_subsystem.getDriveDirection())
// 			m_subsystem.setDriveDirection(false);
// 		else
// 		m_subsystem.setDriveDirection(true);
// 	}

// 	// Called every time the scheduler runs while the command is scheduled.
// 	@Override
// 	public void execute() {
// 	}

// 	// Called once the command ends or is interrupted.
// 	@Override
// 	public void end(boolean interrupted) {
// 	}

// 	// Returns true when the command should end.
// 	@Override
// 	public boolean isFinished() {
// 		return true;
// 	}
// }