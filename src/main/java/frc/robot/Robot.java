// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drive;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
	private RobotContainer m_robotContainer;
	private Command m_autonomousCommand;
	private Thread m_visionThread;
	private boolean red = true;
	private NetworkTable table;
	// private Command m_autonomousCommand;

	private final Timer m_timer = new Timer();

	@Override
	public void robotInit() {
		m_robotContainer = new RobotContainer();
		NetworkTableInstance inst = NetworkTableInstance.getDefault();
		SmartDashboard.getEntry("oi")
			new Thread(
				() -> {
					UsbCamera camera = CameraServer.startAutomaticCapture();
					camera.setResolution(640, 480);
					CvSink cvSink = CameraServer.getVideo();
					CvSource outputStream = CameraServer.putVideo("Processed Image", 640, 480);
					Mat mat = new Mat();

					while(!Thread.interrupted()){
						if(cvSink.grabFrame(mat) == 0){
							outputStream.notifyError(cvSink.getError());
							continue;
						}
						Scalar lowHSVBlue = new Scalar(0, 0, 0);
						Scalar highHSVBlue = new Scalar(0, 0, 0);
						
						Scalar actualValue1; 
						Scalar actualValue2;
						if(m_robotContainer.getTeamColor()){
							
						}
						else{

						}

						Mat thresholdedImage = new Mat();
						Core.inRange(mat, actualValue1, actualValue2, thresholdedImage);

						Mat edges = new Mat();
						Imgproc.Canny(thresholdedImage, edges, 100, 300);

						Imgproc.rectangle(img, pt1, pt2, color);
					}
				}
			)
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		m_robotContainer.setDriveTeleop();
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {
		CommandScheduler.getInstance().run();
		m_robotContainer.driveControl();
		// m_robotContainer.updateShooterSpeed().schedule();
		// m_robotContainer.configureButtonBindings();
	}

	/** This function is run once each time the robot enters autonomous mode. */
	@Override
	public void autonomousInit() {
		m_timer.reset();
		m_timer.start();
		m_robotContainer.setDriveTeleop();
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void testInit() {
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void simulationInit() {
	}

	@Override
	public void simulationPeriodic() {
	}
}
