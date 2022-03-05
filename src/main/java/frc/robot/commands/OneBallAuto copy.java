// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer.AutoPosition;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.util.GeomUtil;

public class OneBallAuto extends SequentialCommandGroup {

	/** Creates a new TwoCargoAuto. */
	
	public OneBallAuto(Intake intake, Shooter shooter) {
						// Create a voltage constraint to ensure we don't accelerate too fast
		addCommands(deadline(
				new ChangeShooterSpeed(shooter, 0.7),
				// new ToggleIntakeBelt(intake),
				new WaitCommand(3.0),
				
                new ToggleIntakeBelt(intake),

				new ChangeShooterSpeed(shooter, 0),
				// sequence(
				// 		sequence(
				// 				new DriveCommand(drive, 0.0,
				// 						List.of(startingPose, cargoPositions.get(position)), 0.0,
				// 						false),
				// 				new DriveCommand(drive, 0.0,
				// 						List.of(cargoPositions.get(position),
				// 								shootPositions.get(position)),
				// 						0.0, true)).deadlineWith(
				// 								new ToggleIntake(intake)),
				// 		new ToggleIntakeBelt(intake),
				// 		new WaitCommand(1.0),
				// 		new ToggleIntake(intake),
				// 		new ToggleIntakeBelt(intake),
				// 		new ChangeShooterSpeed(shooter, 1)),
				// 		new WaitCommand(3.0),
				// 		new ToggleIntakeBelt(intake),
				// 		new WaitCommand(3.0),
				new ChangeShooterSpeed(shooter, 0)));
	}

}