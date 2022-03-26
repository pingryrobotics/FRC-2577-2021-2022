package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
	private CANSparkMax roller;
	private boolean on = false;
	private boolean onBelt = false;
	private CANSparkMax belt;
	private double beltSpeed = Constants.kBeltSpeed;

	/**
	 * Creates a new ExampleSubsystem.
	 */
	public Intake(CANSparkMax rollerSpark, CANSparkMax beltSpark) {
		roller = rollerSpark;
		roller.enableVoltageCompensation(12);
		belt = beltSpark;
		belt.enableVoltageCompensation(12);
		roller.setInverted(true);
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
			belt.set(beltSpeed);
		} else {
			belt.set(0);
		}
	}

	public void flipDirectionRollers() {
		roller.setInverted(!roller.getInverted());
	}

	public void toggleStart() {
		on = !on;
	}

	public void toggleBeltStart() {
		onBelt = !onBelt;
	}

	public void turnOff() {
		onBelt = false;
		on = false;
	}

	public boolean isOn() {
		return (on && onBelt);
	}

	public void flipDirectionBelt() {
		belt.setInverted(!belt.getInverted());
		if (belt.getInverted()) {
			// beltSpeed = 0.1;
			beltSpeed = Constants.kBeltSpeed;
		} else {
			beltSpeed = Constants.kBeltSpeed;
		}
	}
}