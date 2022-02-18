package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ExampleSubsystem;

/** An example command that uses an example subsystem. */
public class ChangeClimberSpeed extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final Climber m_subsystem;
	private boolean direc;

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public ChangeClimberSpeed(Climber subsystem, boolean direc) {
		m_subsystem = subsystem;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(subsystem);
		this.direc = direc;
	}

	@Override
	public void initialize() {
		m_subsystem.set((direc ? 1 : -1) * Constants.kClimberSpeed);
	}

	@Override
	public void end(boolean interrupted) {
	}
}