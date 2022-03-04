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
	private CANSparkMax outtake1;
	private CANSparkMax outtake2;
	private double speed = 0;

	/**
	 * Creates a new ExampleSubsystem.
	 */
	public Shooter(CANSparkMax outtake1, CANSparkMax outtake2) {
		this.outtake1 = outtake1;
		this.outtake2 = outtake2;
		outtake1.enableVoltageCompensation(12);
		outtake2.enableVoltageCompensation(12);
		outtake1.setInverted(false);
		outtake2.setInverted(false);
	}

	@Override
	public void periodic() {
		outtake1.set(speed);
		outtake2.set(speed);
	}

	public void flipDirection() {
		outtake1.setInverted(!outtake1.getInverted());
		outtake2.setInverted(!outtake2.getInverted());
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