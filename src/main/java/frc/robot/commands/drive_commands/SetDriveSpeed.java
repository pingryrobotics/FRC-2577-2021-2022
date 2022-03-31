package frc.robot.commands.drive_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;

/** An example command that uses an example subsystem. */
public class SetDriveSpeed extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final DifferentialSubsystem m_subsystem;
	private final double speed;

	/**
	* Creates a new ExampleCommand.
	*
	* @param subsystem The subsystem used by this command.
	*/
	public SetDriveSpeed(DifferentialSubsystem subsystem, double speed) {
		m_subsystem = subsystem;
		this.speed = speed;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(subsystem);
	}

	@Override
	public void initialize() {
		m_subsystem.setSpeedMultiplier(speed);
		// m_subsystem.invertExtendable();
		// m_subsystem.set((direc ? 1 : -1) * Constants.kClimberSpeed);
	}
}