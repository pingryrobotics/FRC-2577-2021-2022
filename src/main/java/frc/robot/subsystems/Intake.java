package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
	private CANSparkMax roller;
	private boolean on = false;
	private boolean onBelt = false;
	private CANSparkMax belt;

	/**
	 * Creates a new ExampleSubsystem.
	 */
	public Intake(CANSparkMax rollerSpark, CANSparkMax beltSpark) {
		roller = rollerSpark;
		roller.enableVoltageCompensation(12);
		belt = beltSpark;
		belt.enableVoltageCompensation(12);
		roller.setInverted(false);
		belt.setInverted(false);
	}

	@Override
	public void periodic() {
		if (on) {
			roller.set(Constants.kIntakeSpeed);
		} else {
			roller.set(0);
		}
		if (onBelt) {
			belt.set(Constants.kBeltSpeed);
		} else {
			belt.set(0);
		}
	}

	public void flipDirection() {
		roller.setInverted(!roller.getInverted());
	}

	public void toggleStart() {
		on = !on;
	}

	public void toggleStartBelt() {
		onBelt = !onBelt;
	}

	public void flipDirectionBelt() {
		belt.setInverted(!belt.getInverted());
	}
}