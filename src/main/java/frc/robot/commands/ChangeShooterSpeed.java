package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter;

/** An example command that uses an example subsystem. */
public class ChangeShooterSpeed extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final Shooter m_subsystem;
	private double speed;

	/**
	 * Creates a new ChangeShooterSpeed.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public ChangeShooterSpeed(Shooter subsystem, double speed) {
		m_subsystem = subsystem;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(subsystem);
		this.speed = speed;
	}

	@Override
	public void initialize() {
		m_subsystem.setDesiredSpeed(speed);
	}

	@Override
	public void execute() {
	}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void end(boolean interrupted) {
		if (interrupted)
			m_subsystem.setDesiredSpeed(0);
	}
}