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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeLift extends SubsystemBase {
	private CANSparkMax m_flipDownArm;

	/**
	 * Creates a new ExampleSubsystem.
	 */
	public IntakeLift(CANSparkMax armMotor) {
		m_flipDownArm = armMotor;
		m_flipDownArm.enableVoltageCompensation(12);
		m_flipDownArm.setInverted(false);

		m_flipDownArm.setIdleMode(IdleMode.kBrake);
		
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Intake Arm", m_flipDownArm.getEncoder().getPosition());
	}

	// public void flipDown() {
		// m_flipDownArm.getEncoder().setPosition(Constants.kFlipUpArmPosition);
		// while (m_flipDownArm.getEncoder().getPosition() > Constants.kFlipDownArmPosition) {
			// m_flipDownArm.set(-Constants.kFlipDownArmSpeed);
		// }
		// m_flipDownArm.set(0);
	// }
	
	// public void flipUp() {
		
		// while (m_flipDownArm.getEncoder().getPosition() < Constants.kFlipUpArmPosition) {
			
		// }
		// m_flipDownArm.set(0);
	// }
	public void setPower(double speed){
		m_flipDownArm.set(speed);
	}
	public void powerUpwards() {
		m_flipDownArm.set(Constants.kFlipUpArmSpeed);
	}

	public void powerDownwards() {
		m_flipDownArm.set(-Constants.kFlipDownArmSpeed);
	}

	public void powerOff() {
		m_flipDownArm.set(0);
	}

	public double getLiftPosition() {
		return m_flipDownArm.getEncoder().getPosition();
	}

	public void zeroLift() {
		m_flipDownArm.getEncoder().setPosition(0);
	}

	public void holdUp() {
		m_flipDownArm.set(0.05);
	}

	// public void holdDown() {
	// 	m_flipDownArm.set(-0.05);
	// }
}