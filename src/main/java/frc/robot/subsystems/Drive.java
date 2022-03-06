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

public class Drive extends SubsystemBase {
  // The motors on the left side of the drive.

  private MotorControllerGroup m_leftMotors;

  // The motors on the right side of the drive.
  private MotorControllerGroup m_rightMotors;

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The left-side drive encoder

  // private final RelativeEncoder m_leftEncoder;
  // private final RelativeEncoder m_rightEncoder;

  // The gyro sensor
	private final ADIS16470_IMU m_imu = new ADIS16470_IMU(); // 4 seconds for automatic calibration

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  private boolean isDriveDirectionForwards = true;
  private double speedMultiplier = 1;


  /** Creates a new DriveSubsystem. */
  public Drive() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotors.setInverted(true);

    // Sets the distance per pulse for the encoders
    // m_leftEncoder.setPositionConversionFactor(Constants.kDistancePerWheelRevolutionMeters*Constants.kDriveTrainGearReduction);
    // m_rightEncoder.setPositionConversionFactor(Constants.kDistancePerWheelRevolutionMeters*Constants.kDriveTrainGearReduction);

    // m_leftEncoder.setVelocityConversionFactor(Constants.kDistancePerWheelRevolutionMeters*Constants.kDriveTrainGearReduction/60.0);
    // m_rightEncoder.setVelocityConversionFactor(Constants.kDistancePerWheelRevolutionMeters*Constants.kDriveTrainGearReduction/60.0);
// 
    // leftMotor1.burnFlash();
    // leftMotor2.burnFlash();
    // rightMotor1.burnFlash();
    // rightMotor2.burnFlash();

    // m_leftMotors =
    // new MotorControllerGroup(
        // leftMotor1, leftMotor2);

// The motors on the right side of the drive.
    // m_rightMotors =
    // new MotorControllerGroup(
    //     rightMotor1, rightMotor2);


    // resetEncoders();
    m_odometry = new DifferentialDriveOdometry(new Rotation2d(Math.toRadians(m_imu.getAngle()))); // wrong axis?
																								  // default is y	
  }

  public Drive(MotorControllerGroup a, MotorControllerGroup b) {
		m_leftMotors = a;
		m_rightMotors = b;
    m_odometry = new DifferentialDriveOdometry(new Rotation2d(Math.toRadians(m_imu.getAngle()))); // wrong axis?
	}

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    // m_odometry.update(
        // new Rotation2d(Math.toRadians(m_imu.getAngle())), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	public void setPose(Pose2d pose) {
		m_odometry.resetPosition(pose, new Rotation2d(m_imu.getAngle() * -1));
	}

  /**
   * Returns the current wheel speeds of the robot.
   *
  //  * @return The current wheel speeds.
  //  */
  // public DifferentialDriveWheelSpeeds getWheelSpeeds() {
  //   return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  // }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
  //  */
  // public void resetOdometry(Pose2d pose) {
  //   resetEncoders();
  //   m_odometry.resetPosition(pose, new Rotation2d(Math.toRadians(Math.toRadians(m_imu.getAngle()))));
  // }

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

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  // /** Resets the drive encoders to currently read a position of 0. */
  // public void resetEncoders() {
  //   m_leftEncoder.setPosition(0);
  //   m_rightEncoder.setPosition(0);
  // }

  // /**
  //  * Gets the average distance of the two encoders.
  //  *
  //  * @return the average of the two encoder readings
  //  */
  // public double getAverageEncoderDistance() {
  //   return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
  // }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  // public RelativeEncoder getLeftEncoder() {
  //   return m_leftEncoder;
  // }

  // /**
  //  * Gets the right drive encoder.
  //  *
  //  * @return the right drive encoder
  //  */
  // public RelativeEncoder getRightEncoder() {
  //   return m_rightEncoder;
  // }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_imu.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return (new Rotation2d(Math.toRadians(m_imu.getAngle()))).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_imu.getRate();
  }
  
}