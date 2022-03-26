package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import frc.robot.Constants;

public class Indexer {
    private CANSparkMax indexer;
	private boolean on = false;
	private boolean onBelt = false;
	private CANSparkMax belt;
	private double beltSpeed = Constants.kBeltSpeed;

	/**
	 * Creates a new ExampleSubsystem.
	 */
	public Indexer(CANSparkMax indexerMotor) {
		indexer = indexerMotor;
		indexer.enableVoltageCompensation(12);

	}

	
}
