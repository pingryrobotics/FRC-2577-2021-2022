package frc.robot.commands.climb_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.RotatingClimber;
import frc.robot.subsystems.ExampleSubsystem;

/** An example command that uses an example subsystem. */
public class RotatingClimb extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final RotatingClimber m_subsystem;
	private final double speed;

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public RotatingClimb(RotatingClimber subsystem, double speed) {
		m_subsystem = subsystem;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(subsystem);
		this.speed = speed;
	}

	@Override
	public void initialize() {
		m_subsystem.setRotatingSpeed(speed);
		// m_subsystem.invertExtendable();
		// m_subsystem.set((direc ? 1 : -1) * Constants.kClimberSpeed);
	}

	@Override
	public void end(boolean interrupted) {
		m_subsystem.setRotatingSpeed(0);
	}
}