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
	private CANSparkMax outtake;
	private double speed = 1;
	private int flip = 1;

	/**
	 * Creates a new ExampleSubsystem.
	 */
	public Shooter(CANSparkMax outtake) {
		outtake.enableVoltageCompensation(12);
	}

	@Override
	public void periodic() {
		outtake.set(flip * speed);
	}

	public void flipDirection() {
		flip *= -1;
	}

	public void setDesiredSpeed(double speed) {
		this.speed = speed;
	}

	// public void setDesiredAngle(double desiredAngle) {
	// 	angle = desiredAngle;
	// 	if (desiredAngle > 1) {
	// 		angle = 1;
	// 	} else if (desiredAngle < 0) {
	// 		angle = 0;
	// 	}
	// }
}