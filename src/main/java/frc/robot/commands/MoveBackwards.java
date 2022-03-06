package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

/** An example command that uses an example subsystem. */
public class MoveBackwards extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final DifferentialSubsystem m_subsystem;
	private final long millis;
	private long targetTime;
	private double power;

	/**
	* Creates a new ExampleCommand.
	*
	* @param subsystem The subsystem used by this command.
	*/
	public MoveBackwards(DifferentialSubsystem subsystem, int millis, double power) {
		m_subsystem = subsystem;
		this.millis = millis;
		this.power = power;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(subsystem);
	}

	@Override
	public void initialize() {
		m_subsystem.tankDrive(power, power);
		this.targetTime = System.currentTimeMillis() + millis;
		

		// m_subsystem.invertExtendable();
		// m_subsystem.set((direc ? 1 : -1) * Constants.kClimberSpeed);
	}

	public void execute() {
		m_subsystem.tankDrive(power, power);
	}

	public boolean isFinished() {
		return (System.currentTimeMillis() > targetTime);

	}

	public void end() {
		m_subsystem.tankDrive(.5, .5);
	}


}