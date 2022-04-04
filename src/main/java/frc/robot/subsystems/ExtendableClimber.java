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

public class ExtendableClimber extends SubsystemBase {
	private CANSparkMax m_extendableArm;

	/**
	 * Creates a new ExampleSubsystem.
	 */
	public ExtendableClimber(CANSparkMax climbMotor) {
		m_extendableArm = climbMotor;
		m_extendableArm.enableVoltageCompensation(12);
		// m_extendableArm.setSoftLimit(SoftLimitDirection.kForward, Constants.kSlideLimit);
		// m_extendableArm.setSoftLimit(SoftLimitDirection.kReverse, 0);
		m_extendableArm.setInverted(false);

		m_extendableArm.setIdleMode(IdleMode.kBrake);
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

	public void invertExtendable() {
		m_extendableArm.setInverted(!m_extendableArm.getInverted());
	}


	public void setExtendableSpeed(double speed) {
		m_extendableArm.set(speed);
	}
	public void toggleRotate() {
		
	}
}