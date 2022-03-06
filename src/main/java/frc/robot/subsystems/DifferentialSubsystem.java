package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DifferentialSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.

  private MotorControllerGroup m_leftMotors;

  // The motors on the right side of the drive.
  private MotorControllerGroup m_rightMotors;

  // The robot's drive
  private DifferentialDrive m_drive;

  // The left-side drive encoder

  // private final RelativeEncoder m_leftEncoder;
  // private final RelativeEncoder m_rightEncoder;

  // The gyro sensor
	private final ADIS16470_IMU m_imu = new ADIS16470_IMU(); // 4 seconds for automatic calibration

  private boolean isDriveDirectionForwards = true;
  private double speedMultiplier = 1;


  /** Creates a new DriveSubsystem. */
  public DifferentialSubsystem() {
   
  }

  public DifferentialSubsystem(MotorControllerGroup a, MotorControllerGroup b, DifferentialDrive differentialDrive) {
		m_leftMotors = a;
		m_rightMotors = b;
    m_drive = differentialDrive;
	}

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    // m_odometry.update(
        // new Rotation2d(Math.toRadians(m_imu.getAngle())), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
  }


  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void tankDrive(double left, double right) {

    // value is -1 to 1, so reverse so its 1 to -1, add 1 so its 2 to 1, divide by 2 so its 1 to 0
    // setSpeedMultiplier(multiplier);
    double multipliedLeft = left * speedMultiplier;
    double multipliedRight = right * speedMultiplier;
    if (isDriveDirectionForwards) {
      m_drive.tankDrive(multipliedLeft, multipliedRight);
    } else {
      m_drive.tankDrive(-multipliedRight, -multipliedLeft);
    }
  }

  public double getDriveSpeed() {return speedMultiplier;}
  /**
   * Sets the forwards direction of the drivetrain
   * @param isForwards if true, forwards will be intake, otherwise its reversed
   */
  public void setDriveDirection(boolean isForwards) {
    this.isDriveDirectionForwards = isForwards;
  }

  public boolean getDriveDirection(){
      return isDriveDirectionForwards;
  }
  /**
   * Sets the number to multiply the drive speeds by.
   * @param multiplier the multiplier
   */
  public void setSpeedMultiplier(double multiplier) {
    this.speedMultiplier = multiplier;
  }
  
}