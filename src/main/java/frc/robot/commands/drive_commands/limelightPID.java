package frc.robot.commands.drive_commands;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;

public class limelightPID extends CommandBase{
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Drive m_subsystem;
    private Timer timer;
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight"); 
	private double kP = 0.05;
	private double kPDistance = 0.1;
	private double minValue = 0.05;
    private double seconds;


    public limelightPID(Drive subsystem, double seconds) {
		m_subsystem = subsystem;
        this.seconds = seconds;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(subsystem);
	}

	@Override
	public void initialize() {
        timer.start();
	}

	public void execute() {
		if(table.getEntry("tv").getDouble(0.0) == 1){
            double tx = table.getEntry("tx").getDouble(0.0);
            double ty = table.getEntry("ty").getDouble(0.0);
            double distance_error = table.getEntry("ty").getDouble(0.0);
            double headingError = -table.getEntry("tx").getDouble(0.0);
            double steeringAdjust = 0.0;
            if(tx > 0){
                steeringAdjust = kP*headingError - minValue;
            }
            else if(tx < 1.0){
                steeringAdjust = kP*headingError + minValue;
            }
            double distanceAdjust = kPDistance * distance_error;
            m_subsystem.tankDrive(steeringAdjust + distanceAdjust, -steeringAdjust + distanceAdjust);
        }
		
	}
    
    @Override
    public boolean isFinished(){
        return timer.get() >= seconds;
    }
}
