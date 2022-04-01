package frc.robot.commands.drive_commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;

public class PIDTurn extends CommandBase{
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final Drive m_subsystem;
	private double initialHeading;
    private double headingToGo;
	private double kPTurning = 0.05;
	private double minValue = 0.05;

	/**
	* Creates a new ExampleCommand.
	*
	* @param subsystem The subsystem used by this command.
	*/
	public PIDTurn(Drive subsystem, double angle) {
		m_subsystem = subsystem;
		initialHeading = subsystem.getHeading();
        headingToGo = initialHeading + angle;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(subsystem);
	}

	@Override
	public void initialize() {
	}

	public void execute() {
		double headingError = (m_subsystem.getHeading() - headingToGo);
		double steeringAdjust = 0;
		if(headingError > 0){
			steeringAdjust = kPTurning*headingError + minValue;
		}
		else if(headingError < 0){
			steeringAdjust = kPTurning*headingError - minValue;
		}
		m_subsystem.tankDrive(steeringAdjust, -steeringAdjust);
		
	}
    
    @Override
    public boolean isFinished(){
        return headingToGo - m_subsystem.getHeading() <= 3 && headingToGo - m_subsystem.getHeading() >= -3;
    }

}
