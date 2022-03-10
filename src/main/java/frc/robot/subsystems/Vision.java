package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
	private Shooter shooter;
	private boolean shootOn = false;
    private NetworkTable table;
    private NetworkTableEntry tv;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;
    private NetworkTableEntry ts;
    private NetworkTableEntry tl;
    private NetworkTableEntry tshort;
    private NetworkTableEntry tlong;
    private NetworkTableEntry thor;
    private NetworkTableEntry tvert;
    private NetworkTableEntry getpipe;
    private NetworkTableEntry camtran;
    private NetworkTableEntry ledMode;
    private NetworkTableEntry camMode;
    private NetworkTableEntry pipeline;
    private NetworkTableEntry stream;
    private NetworkTableEntry snapshot;
    public double latency;
    public LedMode givenLedMode;
    public int givenPipeline;
    public double xOffset;
    public double yOffset;
    public double area;
    public LedMode desiredLedMode = LedMode.OFF;
    public int desiredPipeline = 0;
    public int desiredStream = 2;
    public int desiredSnapshot = 0;
    private boolean mOutputsHaveChanged = true;
    private boolean mSeesTarget = false;
    private final double limelightMountAngleDegrees = 0.0;
    private final double limelightHeightInches = 29.5;
	private final double goalHeightInches = 104.0;

    public enum LedMode{
        PIPELINE, OFF, BLINK, ON
    }
    public enum CamMode{
        VISION, DRIVER
    }
    /**
     * Creates a new Limelight subsystem.
     */
    public Vision(Shooter shooter) {
        table = NetworkTableInstance.getDefault().getTable("limelight");
		this.shooter = shooter;
		ty = table.getEntry("ty");

        // tv = table.getEntry("tv");
        // tx = table.getEntry("tx");
        // ta = table.getEntry("ta");
        // ts = table.getEntry("ts");
        // tl = table.getEntry("tl");
        // tshort = table.getEntry("tshort");
        // tlong = table.getEntry("tlong");
        // thor = table.getEntry("thor");
        // tvert = table.getEntry("tvert");
        // getpipe = table.getEntry("getpipe");
        // camtran = table.getEntry("camtran");
        // ledMode = table.getEntry("ledMode");
        // camMode = table.getEntry("camMode");
        // pipeline = table.getEntry("pipeline");
        // stream = table.getEntry("stream");
        // snapshot = table.getEntry("snapshot");
    }

	/**
	 * Get power based on distance from target
	 */
	public double getPower(double distanceInches) {
		// polynomial regression of distance vs. power
		// y = ax^5 + bx^4 + etc.
		// distance should be measured 
		double x = distanceInches;
		return x;
	}

    @Override
	public void periodic() {
		double targetOffsetAngle_Vertical = ty.getDouble(1e9);
		if (targetOffsetAngle_Vertical == 1e9) { // if the target is not visible
			mSeesTarget = false;
		} else {
			mSeesTarget = true;
		}
		double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
		double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
		double distanceFromLimelightToGoalInches = (goalHeightInches - limelightHeightInches)
				/ Math.tan(angleToGoalRadians);
		if (shootOn && mSeesTarget) {
			shooter.setDesiredSpeed(getPower(distanceFromLimelightToGoalInches));
		}

		// SmartDashboard.putBoolean("Limelight Has Target", mSeesTarget);
		// SmartDashboard.putNumber("Limelight Pipeline Latency (ms)", latency);
		// latency = tl.getDouble(0) / 1000.0 + Constants.kImageCaptureLatency;
		// givenLedMode = LedMode.values()[(int) ledMode.getDouble(1.0)];
		// givenPipeline = (int) pipeline.getDouble(0.0);
		// xOffset = tx.getDouble(0.0);
		// yOffset = ty.getDouble(0.0);
		// area = ta.getDouble(0.0);
		// mSeesTarget = tv.getDouble(0) == 1.0;
		// if (givenLedMode != desiredLedMode || givenPipeline != desiredPipeline){
		//     mOutputsHaveChanged = true;
		// }
		// if(mOutputsHaveChanged){
		//     ledMode.setNumber((int)desiredLedMode);
		//     camMode.setNumber((int) camMode.getNumber(-1));
		//     pipeline.setNumber(desiredPipeline);
		//     stream.setNumber(desiredStream);
		//     snapshot.setNumber(desiredSnapshot);
		// }
	}
	
	public void toggleAutoShoot() {
		shootOn = !shootOn;
	}
    public void setLed(LedMode mode) {
        desiredLedMode = mode;
    }

    public void setPipeline(int mode) {
        desiredPipeline = mode;
    }
    public void triggerOutputs() {
        mOutputsHaveChanged = true;
    }
    public int getPipeline() {
        return givenPipeline;
    }
    // public int getLedMode() {
        // return givenLedMode;
    // }
    public boolean seesTarget() {
        return mSeesTarget;
    }
  }
