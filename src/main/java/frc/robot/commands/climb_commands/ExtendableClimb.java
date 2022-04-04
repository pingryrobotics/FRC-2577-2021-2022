package frc.robot.commands.climb_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.RotatingClimber;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ExtendableClimber;

/** An example command that uses an example subsystem. */
public class ExtendableClimb extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final ExtendableClimber m_subsystem;

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public ExtendableClimb(ExtendableClimber subsystem) {
		m_subsystem = subsystem;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(subsystem);
	}

	@Override
	public void initialize() {
		// m_subsystem.invertExtendable();
		m_subsystem.setExtendableSpeed(-Constants.kClimberSpeed);
		// m_subsystem.set((direc ? 1 : -1) * Constants.kClimberSpeed);
	}

	@Override
	public void end(boolean interrupted) {
		// m_subsystem.invertExtendable();
		m_subsystem.setExtendableSpeed(0);
	}
}