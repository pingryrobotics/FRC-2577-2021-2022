// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;

// import edu.wpi.first.wpilibj.I2C;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import com.revrobotics.ColorSensorV3;
// import frc.robot.Constants;

// public class ColorSensor extends SubsystemBase {
// 	private final I2C.Port i2cPort = I2C.Port.kOnboard;
// 	private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
// 	private Intake intake;
// 	private boolean passBall = false;

// 	/**
// 	 * Creates a new ExampleSubsystem.
// 	 */
// 	public ColorSensor(Intake intake) {
// 		this.intake = intake;	
// 	}

// 	@Override
// 	public void periodic() {
// 		if (intake.isOn() && (m_colorSensor.getProximity() < Constants.kProximityThreshold || m_colorSensor.getColor().equals(Color.kBlue) || m_colorSensor.getColor().equals(Color.kRed))) {
// 			if (!passBall) {
// 				intake.turnOff();
// 			}
// 		}
// 	}

// 	public void toggleAllowBall() {
// 		passBall = !passBall;
// 	}

// }