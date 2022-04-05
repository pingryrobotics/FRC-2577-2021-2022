// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import javax.swing.text.html.ListView;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
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
		SmartDashboard.getEntry("oi");
			m_visionThread = new Thread(new Runnable() {

				@Override
				public void run() {
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
						Scalar lowHSVBlue = new Scalar(203, 51, 35);
						Scalar highHSVBlue = new Scalar(223, 71, 55);

						Scalar lowHSVRed = new Scalar(0, 74, 64);
						Scalar highHSVRed = new Scalar(12, 94, 84);
						
						Scalar actualHSVLow; 
						Scalar actualHSVHigh;
						if(m_robotContainer.getTeamColor()){
							actualHSVLow = lowHSVRed;
							actualHSVHigh = highHSVRed;
						}
						else{
							actualHSVLow = lowHSVBlue;
							actualHSVHigh = highHSVBlue;
						}

						Mat thresholdedImage = new Mat();
						Core.inRange(mat, actualHSVLow, actualHSVHigh, thresholdedImage);

						Mat edges = new Mat();
						Imgproc.Canny(thresholdedImage, edges, 100, 300);

						List<MatOfPoint> contours = new ArrayList<>();
						Mat hierarchy = new Mat();
						
						Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

						int sz = contours.size();

						MatOfPoint2f[] contoursPoly = new MatOfPoint2f[sz];
						Rect biggestRect = new Rect();
						Rect[] boundRects = new Rect[sz];
						for (int i = 0; i < sz; i++) {
							contoursPoly[i] = new MatOfPoint2f();
							Imgproc.approxPolyDP(
								new MatOfPoint2f(contours.get(i).toArray()),
								contoursPoly[i],
								3,
								true);
							boundRects[i] = Imgproc.boundingRect(new MatOfPoint2f(contoursPoly[i].toArray()));
							if (boundRects[i].area() >= biggestRect.area()) {
								biggestRect = boundRects[i];
							}
						}

						Imgproc.rectangle(thresholdedImage, biggestRect, new Scalar(255, 0, 0), 4);
						int rectCenterX = biggestRect.x + (biggestRect.width / 2);
						
						


					
				}
				
			};


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
