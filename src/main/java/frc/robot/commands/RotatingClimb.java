package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ExampleSubsystem;

/** An example command that uses an example subsystem. */
public class RotatingClimb extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final Climber m_subsystem;

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public RotatingClimb(Climber subsystem) {
		m_subsystem = subsystem;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(subsystem);
	}

	@Override
	public void initialize() {
		m_subsystem.setRotatingSpeed(Constants.kRotatingSpeed);
		// m_subsystem.invertExtendable();
		// m_subsystem.set((direc ? 1 : -1) * Constants.kClimberSpeed);
	}

	@Override
	public void end(boolean interrupted) {
		m_subsystem.setRotatingSpeed(0);
	}
}