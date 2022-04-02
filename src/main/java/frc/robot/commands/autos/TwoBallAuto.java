package frc.robot.commands.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive_commands.DriveCommand;
import frc.robot.commands.drive_commands.DriveForwardPID;
import frc.robot.commands.drive_commands.PIDTurn;
import frc.robot.commands.drive_commands.SetDriveSpeed;
import frc.robot.commands.drive_commands.limelightPID;
import frc.robot.commands.intake_commands.IntakeLiftDown;
import frc.robot.commands.intake_commands.IntakeLiftDownAutonomous;
import frc.robot.commands.intake_commands.SetBeltEnabled;
import frc.robot.commands.intake_commands.SetIntakeEnabled;
import frc.robot.commands.shooter_commands.ChangeShooterSpeed;
import frc.robot.commands.shooter_commands.IndexerOut;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeLift;
import frc.robot.subsystems.Shooter;
import frc.robot.util.GeomUtil;

public class TwoBallAuto extends SequentialCommandGroup {

    public TwoBallAuto(Drive drive, Intake intake, IntakeLift intakeLift, Shooter shooter, Indexer indexer) {
        Pose2d startingPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        drive.setPose(startingPose);
        drive.zeroHeading();
        // addCommands(sequence(
        //     new DriveCommand(drive, 0.0, List.of(startingPose, 
        //     new Pose2d(new Translation2d(1, 0.0), new Rotation2d(Math.PI, 0))), 0.0,  false)
        // ));

        addCommands(sequence(
            new IntakeLiftDownAutonomous(intakeLift, 0.5),
            new SetIntakeEnabled(intake, true),
            new SetBeltEnabled(intake, true),
            new DriveForwardPID(drive, 1.0),
            new PIDTurn(drive, 180),
            new ChangeShooterSpeed(shooter, 0.7),
            new limelightPID(drive, 3),
            new IndexerOut(indexer),
            new WaitCommand(1),
            new IndexerOut(indexer),
            new ChangeShooterSpeed(shooter, 0)
        ));


    }
    
}
