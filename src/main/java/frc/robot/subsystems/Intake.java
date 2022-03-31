package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
	private CANSparkMax roller;
	private boolean intakeOn = false;
	private boolean beltOn = false;
	private CANSparkMax belt;
	private double beltSpeed = Constants.kBeltSpeed;
	private final boolean beltForwardsBoolean = true;
	private final boolean intakeForwardsBoolean = false;

	/**
	 * Creates a new ExampleSubsystem.
	 */
	public Intake(CANSparkMax rollerSpark, CANSparkMax beltSpark) {
		roller = rollerSpark;
		roller.enableVoltageCompensation(12);
		belt = beltSpark;
		belt.enableVoltageCompensation(12);
		roller.setInverted(intakeForwardsBoolean);
		belt.setInverted(beltForwardsBoolean);
	}

	@Override
	public void periodic() {
		if (intakeOn) {
			roller.set(Constants.kIntakeSpeed);
		} else {
			roller.set(0);
		}
		if (beltOn) {
			belt.set(beltSpeed);
		} else {
			belt.set(0);
		}
	}

	public void flipDirectionRollers() {
		roller.setInverted(!roller.getInverted());
	}

	public void toggleStart() {
		intakeOn = !intakeOn;
	}

	public void toggleBeltStart() {
		beltOn = !beltOn;
	}

	public void turnOff() {
		beltOn = false;
		intakeOn = false;
	}

	public boolean isOn() {
		return (intakeOn && beltOn);
	}

	public void setAllEnabled(boolean turnOn) {
		intakeOn = beltOn = turnOn;
	}
	
	public void setBeltEnabled(boolean turnOn) {
		beltOn = turnOn;
	}

	public void setIntakeEnabled(boolean turnOn) {
		intakeOn = turnOn;
	}

	
	public void setAllDirection(boolean turnForwards) {
		belt.setInverted((turnForwards) ? beltForwardsBoolean : !beltForwardsBoolean);
		roller.setInverted((turnForwards) ? intakeForwardsBoolean : !intakeForwardsBoolean);

	}
	
	public void setBeltDirection(boolean turnForwards) {
		belt.setInverted((turnForwards) ? beltForwardsBoolean : !beltForwardsBoolean);
	}

	public void setIntakeDirection(boolean turnForwards) {
		roller.setInverted((turnForwards) ? intakeForwardsBoolean : !intakeForwardsBoolean);
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