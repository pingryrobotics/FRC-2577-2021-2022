/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
	private CANSparkMax m_linearSlide;
	private CANSparkMax m_rotatingArm;
	/**
	 * Creates a new ExampleSubsystem.
	 */
	public Climber(CANSparkMax climbMotor, CANSparkMax armMotor) {
		m_linearSlide = climbMotor;
		m_linearSlide.enableVoltageCompensation(12);
		m_linearSlide.setSoftLimit(SoftLimitDirection.kForward, Constants.kSlideLimit);
		m_linearSlide.setSoftLimit(SoftLimitDirection.kReverse, 0);
		m_rotatingArm = armMotor;
		m_rotatingArm.enableVoltageCompensation(12);
		m_rotatingArm.setSoftLimit(SoftLimitDirection.kForward, Constants.kArmLimit);
		m_rotatingArm.setSoftLimit(SoftLimitDirection.kReverse, 0);
	}


	public Climber(CANSparkMax climbMotor) {
		m_linearSlide = climbMotor;
		m_linearSlide.enableVoltageCompensation(12);
		m_linearSlide.setSoftLimit(SoftLimitDirection.kForward, Constants.kSlideLimit);
		m_linearSlide.setSoftLimit(SoftLimitDirection.kReverse, 0);
	}

	public void set(double speed) {
		m_linearSlide.set(speed);
	}

	// public void setArm(double speed) {
		// m_rotatingArm.set(speed);
	// }

	public void invertExtendable() {
		m_linearSlide.setInverted(true);
	}

	// public void invertRetractable() {
		// m_rotatingArm.setInverted(true);
	// }

	@Override
	public void periodic() {
		// This method will be called once per scheduler run

	}
}