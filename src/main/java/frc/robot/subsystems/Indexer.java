package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import frc.robot.Constants;

public class Indexer {
    private CANSparkMax indexer;
	private boolean on = false;
	private double power = 0.0;
	// private boolean onBelt = false;
	// private CANSparkMax belt;
	private double beltSpeed = Constants.kBeltSpeed;

	/**
	 * Creates a new ExampleSubsystem.
	 */
	public Indexer(CANSparkMax indexerMotor) {
		indexer = indexerMotor;
		indexer.enableVoltageCompensation(12);

	}

	public void periodic() {
		if (on) {
			indexer.set(power * Constants.kIndexerSpeed);
		} else {
			indexer.set(0);
		}
	}

	public void toggleStart() {
		on = !on;
	}
	
	public void reverse() {
		power *= -1;
	}
}
