// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer.AutoPosition;
import frc.robot.commands.drive_commands.ChangeDriveSpeed;
import frc.robot.commands.drive_commands.DriveCommand;
import frc.robot.commands.intake_commands.ToggleBelt;
import frc.robot.commands.intake_commands.ToggleIntake;
import frc.robot.commands.intake_commands.ToggleIntakeAndBelt;
import frc.robot.commands.shooter_commands.ChangeShooterSpeed;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.util.GeomUtil;

public class TwoCargoAuto extends SequentialCommandGroup {
	public static final double shootDurationSecs = 3.0;
	public static final Map<AutoPosition, Pose2d> cargoPositions = Map.of(
        AutoPosition.TARMAC_A,
			Constants.cargoB
					.transformBy(GeomUtil.transformFromTranslation(-0.5, 0.0)),
			AutoPosition.TARMAC_C,
			Constants.cargoD.transformBy(new Transform2d(
					new Translation2d(0.25, 0.0), Rotation2d.fromDegrees(-45.0))),
			AutoPosition.TARMAC_D, 
            Constants.cargoE
					.transformBy(GeomUtil.transformFromTranslation(-0.5, 0.0)));
	public AutoPosition pos;

	/** Creates a new TwoCargoAuto. */
	public TwoCargoAuto(AutoPosition position, Drive drive, Intake intake, Shooter shooter) {
						// Create a voltage constraint to ensure we don't accelerate too fast
		
        System.out.println("two cargo initialized");
        Pose2d startingPose = position.getPose();
		pos = position;
		addCommands(sequence(
				new ChangeShooterSpeed(shooter, 1),
				new ToggleIntakeAndBelt(intake),
				new WaitCommand(3.0),
				new ToggleIntakeAndBelt(intake),	
				new ChangeShooterSpeed(shooter, 0),
						sequence(
								new DriveCommand(drive, 0.0,
										List.of(startingPose, cargoPositions.get(position)), 0.0,
										false),
								new DriveCommand(drive, 0.0,
										List.of(cargoPositions.get(position),
												new Pose2d(new Translation2d(0.1, 0.0), new Rotation2d())),
										0.0, true)).deadlineWith(
												new ToggleIntake(intake)),
						new ToggleBelt(intake), // belt on to get ball in
						new WaitCommand(1.0), 
						new ToggleIntakeAndBelt(intake), // belt off to keep ball in
						new ChangeShooterSpeed(shooter, 1), // get ready to shoot
						new WaitCommand(3.0),
						new ToggleBelt(intake), // shoot
						new WaitCommand(3.0),
				new ChangeShooterSpeed(shooter, 0),
				new ToggleBelt(intake),
				new ChangeDriveSpeed(drive, 0)
        ));
	}


	public static Pose2d calcAimedPose(Pose2d pose) {
		Translation2d vehicleToCenter = Constants.hubCenter.minus(pose.getTranslation());
		Rotation2d targetRotation = new Rotation2d(vehicleToCenter.getX(), vehicleToCenter.getY());
		targetRotation = targetRotation.plus(Rotation2d.fromDegrees(180));
		return new Pose2d(pose.getTranslation(), targetRotation);
	}
}