package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;

/** An example command that uses an example subsystem. */
public class SetDriveDirection extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final DifferentialSubsystem m_subsystem;
	private final boolean isForwards;

	/**
	* Creates a new ExampleCommand.
	*
	* @param subsystem The subsystem used by this command.
	*/
	public SetDriveDirection(DifferentialSubsystem subsystem, boolean isForwards) {
		m_subsystem = subsystem;
		this.isForwards = isForwards;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(subsystem);
	}

	@Override
	public void initialize() {
		m_subsystem.setDriveDirection(isForwards);
		// m_subsystem.invertExtendable();
		// m_subsystem.set((direc ? 1 : -1) * Constants.kClimberSpeed);
	}
}