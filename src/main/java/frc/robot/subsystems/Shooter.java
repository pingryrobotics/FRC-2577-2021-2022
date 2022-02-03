/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
	private CANSparkMax leftFlywheel;
	private CANSparkMax rightFlywheel;
	private CANSparkMax lift;
	private double kFlip = 1;
	private double desiredSpeed = .9;
	private double angle = 0;

	/**
	 * Creates a new ExampleSubsystem.
	 */
	public Shooter(CANSparkMax left, CANSparkMax right, CANSparkMax leadScrew) {
		leftFlywheel = left;
		rightFlywheel = right;
		lift = leadScrew;
		leadScrew.setInverted(false);
		leadScrew.setVoltage(11);
		leftFlywheel.setInverted(true);
		rightFlywheel.setInverted(false);
		leftFlywheel.enableVoltageCompensation(12);
		rightFlywheel.enableVoltageCompensation(12);
	}

	@Override
	public void periodic() {
		leftFlywheel.set(kFlip * desiredSpeed);
		rightFlywheel.set(kFlip * desiredSpeed);
	}

	public void flipDirection() {
		kFlip = -kFlip;
	}

	public void setDesiredSpeed(double speed) {
		desiredSpeed = speed;
	}

	public void setDesiredAngle(double desiredAngle) {
		angle = desiredAngle;
		if (desiredAngle > 1) {
			angle = 1;
		} else if (desiredAngle < 0) {
			angle = 0;
		}
	}
}