/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RotatingClimber extends SubsystemBase {
	private CANSparkMax m_rotatingArm;

	/**
	 * Creates a new ExampleSubsystem.
	 */
	public RotatingClimber(CANSparkMax rotatingMotor) {

		// m_extendableArm.setSoftLimit(SoftLimitDirection.kForward, Constants.kSlideLimit);
		// m_extendableArm.setSoftLimit(SoftLimitDirection.kReverse, 0);
		m_rotatingArm = rotatingMotor; 
		m_rotatingArm.enableVoltageCompensation(12);
		m_rotatingArm.setSoftLimit(SoftLimitDirection.kForward, Constants.kArmLimit);
		m_rotatingArm.setSoftLimit(SoftLimitDirection.kReverse, 0);

		m_rotatingArm.setInverted(false);

		m_rotatingArm.setIdleMode(IdleMode.kBrake);
	}

	// public Climber(CANSparkMax climbMotor) {
	// 	m_extendableArm = climbMotor;
	// 	m_extendableArm.enableVoltageCompensation(12);
	// 	m_extendableArm.setSoftLimit(SoftLimitDirection.kForward, Constants.kSlideLimit);
	// 	m_extendableArm.setSoftLimit(SoftLimitDirection.kReverse, 0);
	// }

	@Override
	public void periodic() {
		// m_extendableArm.set();
		// m_rotatingArm.set();
	}


	public void invertRotating() {
		m_rotatingArm.setInverted(!m_rotatingArm.getInverted());
	}


	public void setRotatingSpeed(double speed) {
		m_rotatingArm.set(speed);
	}

	public void toggleRotate() {
		
	}
}