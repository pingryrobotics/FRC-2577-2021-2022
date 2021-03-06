package frc.robot.commands.intake_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.RotatingClimber;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeLift;

/** An example command that uses an example subsystem. */
public class IntakeLiftUp extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final IntakeLift m_subsystem;

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param m_intake The subsystem used by this command.
	 */
	public IntakeLiftUp(IntakeLift m_intake) {
		m_subsystem = m_intake;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_intake);
	}

	@Override
	public void initialize() {
		m_subsystem.powerUpwards();
		// m_subsystem.invertExtendable();
		// m_subsystem.set((direc ? 1 : -1) * Constants.kClimberSpeed);
	}

	@Override
	public void execute() {
		System.out.println("Lift position: " + m_subsystem.getLiftPosition());
	}

	@Override
	public boolean isFinished() {
		return true;
		// return m_subsystem.getLiftPosition() <= Constants.kFlipUpArmPosition;
	}

	@Override
	public void end(boolean interrupted) {
		// m_subsystem.powerOff();
		// m_subsystem.holdUp();
	}
}