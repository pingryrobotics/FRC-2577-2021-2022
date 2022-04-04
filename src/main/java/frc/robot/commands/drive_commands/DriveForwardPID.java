package frc.robot.commands.drive_commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;

public class DriveForwardPID extends CommandBase{
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final Drive m_subsystem;
    private Timer timer;
    private double seconds;
	private double initialHeading;
	private double kPTurning = 0.05;
	private double minValue = 0.05;

	/**
	* Creates a new ExampleCommand.
	*
	* @param subsystem The subsystem used by this command.
	*/
	public DriveForwardPID(Drive subsystem, double timeout) {
		m_subsystem = subsystem;
        seconds = timeout;
		this.timer = new Timer();
		initialHeading = subsystem.getHeading();
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(subsystem);
	}

	@Override
	public void initialize() {
        timer.start();
	}

	public void execute() {
		double headingError = (m_subsystem.getHeading() - initialHeading);
		double steeringAdjust = 0;
		if(headingError > 0){
			steeringAdjust = kPTurning*headingError + minValue;
		}
		else if(headingError < 0){
			steeringAdjust = kPTurning*headingError - minValue;
		}
		m_subsystem.tankDrive(steeringAdjust + 0.5, -steeringAdjust + 0.5);
		
	}
    
    @Override
    public boolean isFinished(){
        return timer.get() >= seconds;
    }

	@Override
	public void end(boolean interrupted) {
		m_subsystem.tankDrive(0, 0);
	} 

}
