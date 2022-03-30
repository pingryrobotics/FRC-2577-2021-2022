package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
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
		// if (on) {
			// indexer.set(power * Constants.kIndexerSpeed);
		// } else {
			// indexer.set(0);
		// }
		// indexer.set(1);
	}

	// public void toggleStart() {
		// on = !on;
	// }
	// 
	// public void reverse() {
		// power *= -1;
	// }

	public void moveDown() {
		double initialPos = indexer.getEncoder().getPosition();
		while (indexer.getEncoder().getPosition() < initialPos + 15) {
			indexer.set(Constants.kIndexerSpeed);
		}
		indexer.set(0);
	}

	public void moveUp() {
		double initialPos = indexer.getEncoder().getPosition();
		while (indexer.getEncoder().getPosition() > initialPos - 15) {
			indexer.set(-Constants.kIndexerSpeed);
		}
		indexer.set(0);
	}
}
