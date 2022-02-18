package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
	private CANSparkMax roller;
	private boolean on = true;

	/**
	 * Creates a new ExampleSubsystem.
	 */
	public Intake(CANSparkMax rollerSpark) {
		roller = rollerSpark;
		roller.enableVoltageCompensation(12);
	}

	@Override
	public void periodic() {
		if (on) {
			roller.set(Constants.kIntakeSpeed);
		} else {
			roller.set(0);
		}
	}

	public void flipDirection() {
		roller.setInverted(!roller.getInverted());
	}

	public void toggleStart() {
		on = !on;
	}
}