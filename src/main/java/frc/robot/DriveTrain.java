package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

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

public class DriveTrain {
    private WPI_TalonSRX leftMaster = new WPI_TalonSRX(DriveTrainConstants.leftMasterTalonID);
    private WPI_TalonSRX rightMaster = new WPI_TalonSRX(DriveTrainConstants.rightMasterTalonID);
    private WPI_TalonSRX leftFollower = new WPI_TalonSRX(DriveTrainConstants.leftSlaveTalonID);
    private WPI_TalonSRX rightFollower = new WPI_TalonSRX(DriveTrainConstants.rightSlaveTalonID);

    private SpeedControllerGroup leftMotors;
    private SpeedControllerGroup rightMotors;

    private DifferentialDrive differentialDrive;
    private boolean invertedDrive = false;

    public Odometry odometry; // savy users can directly access odometry

    /**
     * Constructor for the drivetrain class
     */
    public DriveTrain() {
        WPI_TalonSRX motors[] = new WPI_TalonSRX[] { leftMaster, rightMaster, leftFollower, rightFollower };
        for (WPI_TalonSRX motor : motors)
            configureMotor(motor);

        leftMotors = new SpeedControllerGroup(leftMaster, leftFollower);
        rightMotors = new SpeedControllerGroup(rightMaster, rightFollower);
        differentialDrive = new DifferentialDrive(leftMotors, rightMotors);
    }

    public void teleopPeriodic(Joystick leftDriverJoystick, Joystick rightDriverJoystick) {
        odometry.teleopPeriodic(); // update odometry information

        if (leftDriverJoystick.getRawButtonReleased(DriveTrainConstants.invertDriveButton))
            invertedDrive = !invertedDrive;

        if (!invertedDrive)
            differentialDrive.tankDrive(leftDriverJoystick.getY(), rightDriverJoystick.getY());
        else
            differentialDrive.tankDrive(-rightDriverJoystick.getY(), -leftDriverJoystick.getY());
    }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more
     * slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        differentialDrive.setMaxOutput(maxOutput);
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
}