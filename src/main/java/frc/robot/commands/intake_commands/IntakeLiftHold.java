package frc.robot.commands.intake_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.RotatingClimber;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeLift;

/** An example command that uses an example subsystem. */
public class IntakeLiftHold extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final IntakeLift m_subsystem;

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param m_intake The subsystem used by this command.
	 */
	public IntakeLiftHold(IntakeLift m_intake) {
		m_subsystem = m_intake;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_intake);
	}

	@Override
	public void initialize() {
		// m_subsystem.powerDownwards();
		m_subsystem.holdUp();
		// m_subsystem.invertExtendable();
		// m_subsystem.set((direc ? 1 : -1) * Constants.kClimberSpeed);
	}
	

	@Override
	public boolean isFinished() {
		// return !(m_subsystem.getLiftPosition() > Constants.kFlipDownArmPosition) ;
		return true;
	}

	@Override
	public void end(boolean interrupted) {
		// m_subsystem.powerOff();
	}
}