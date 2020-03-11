package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

class DriveTrainConstants {
    // PID Constants
    public static final double kP = 0.25;
    public static final double kI = 0.001;
    public static final double kD = 20;
    public static final double kF = 1023.0 / 7200.0;

    // Talon IDs
    public static final int leftMasterTalonID = 2;
    public static final int rightMasterTalonID = 3;
    public static final int leftSlaveTalonID = 4;
    public static final int rightSlaveTalonID = 5;

    // Button Mapping
    public static int invertDriveButton = 4;
}

public class DriveTrain extends SubsystemBase {
    private WPI_TalonSRX leftMaster = new WPI_TalonSRX(DriveTrainConstants.leftMasterTalonID);
    private WPI_TalonSRX rightMaster = new WPI_TalonSRX(DriveTrainConstants.rightMasterTalonID);
    private WPI_TalonSRX leftFollower = new WPI_TalonSRX(DriveTrainConstants.leftSlaveTalonID);
    private WPI_TalonSRX rightFollower = new WPI_TalonSRX(DriveTrainConstants.rightSlaveTalonID);

    private SpeedControllerGroup leftMotors;
    private SpeedControllerGroup rightMotors;

    private AHRS gyro;

    private DifferentialDrive differentialDrive;
    private DifferentialDriveOdometry odometry;

    private boolean invertedDrive = false;

    private static DriveTrain instance;

    /**
     * Constructor for the drivetrain class
     */
    public DriveTrain() {
        WPI_TalonSRX motors[] = new WPI_TalonSRX[] { leftMaster, rightMaster, leftFollow, rightFollow };
        for (WPI_TalonSRX motor : motors)
            configureMotor(motor);

        leftMotors = new SpeedControllerGroup(leftMaster, leftFollow);
        rightMotors = new SpeedControllerGroup(rightMaster, rightFollow);
        differentialDrive = new DifferentialDrive(leftMotors, rightMotors);

        gyro = new AHRS(SPI.Port.kMXP);
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(gyro.getYaw()));
    }

    public void periodic(Joystick leftDriver, Joystick rightDriver) {
        //By forcing this code to only run in teleop, we prevent
        //drivetrain signals from being sent while in autonomous mode

        if (DriverStation.getInstance().isOperatorControl()) { //Only runs this block if in tele-op
            if (leftDriver.getRawButtonReleased(DriveTrainConstants.invertDriveButton))
                invertedDrive = !invertedDrive;

            if (!invertedDrive)
                differentialDrive.tankDrive(leftDriver.getY(), rightDriver.getY());
            else
                differentialDrive.tankDrive(-rightDriver.getY(), -leftDriver.getY());
        }
        //Update the odometry information using our encoders
        //TODO: unit conversions for ticks to meters
        odometry.update(Rotation2d.fromDegrees(gyro.getYaw()),
                leftMaster.getSelectedSensorPosition(), rightMaster.getSelectedSensorPosition());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftMaster.getSelectedSensorVelocity(), rightMaster.getSelectedSensorVelocity());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, Rotation2d.fromDegrees(gyro.getYaw()));
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
        differentialDrive.arcadeDrive(fwd, rot);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     * Useful for preventing voltage sag.
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotors.setVoltage(leftVolts);
        rightMotors.setVoltage(-rightVolts);
        differentialDrive.feed();
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        leftMaster.setSelectedSensorPosition(0);
        rightMaster.setSelectedSensorPosition(0);
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (leftMaster.getSelectedSensorPosition() + rightMaster.getSelectedSensorPosition()) / 2.0;
    }

    /**
     * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        differentialDrive.setMaxOutput(maxOutput);
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * A helper method used to configure drivetrain motors
     *
     * @param driveMotor the motor to configure
     */
    public void configureMotor(WPI_TalonSRX driveMotor) {
        driveMotor.configFactoryDefault();
        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.configOpenloopRamp(0);
        driveMotor.configClosedloopRamp(0);
        driveMotor.configPeakOutputForward(1);
        driveMotor.configPeakOutputReverse(-1);
        driveMotor.config_kP(0, DriveTrainConstants.kP);
        driveMotor.config_kI(0, DriveTrainConstants.kI);
        driveMotor.config_kD(0, DriveTrainConstants.kD);
        driveMotor.config_kF(0, DriveTrainConstants.kF);
    }

    /**
     * A helper method to ensure that no duplicate DriveTrain objects
     * are created anywhere else in this project.
     *
     * @return A singular instance of the DriveTrain class.
     */
    public static DriveTrain getInstance(){
        if (instance == null)
            instance = new DriveTrain();
        return instance;
}