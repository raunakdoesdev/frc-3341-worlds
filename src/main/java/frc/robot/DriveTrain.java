package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

class DriveTrainConstants {
    // PID Constants
    public static double kP = 0.25;
    public static double kI = 0.001;
    public static double kD = 20;
    public static double kF = 1023.0 / 7200.0;

    // Talon IDs
    public static int leftMasterTalonID = 2;
    public static int rightMasterTalonID = 3;
    public static int leftSlaveTalonID = 4;
    public static int rightSlaveTalonID = 5;

    // Button Mapping
    public static int invertDriveButton = 4;
}

public class DriveTrain {
    private WPI_TalonSRX leftMaster = new WPI_TalonSRX(DriveTrainConstants.leftMasterTalonID);
    private WPI_TalonSRX rightMaster = new WPI_TalonSRX(DriveTrainConstants.rightMasterTalonID);
    private WPI_TalonSRX leftFollow = new WPI_TalonSRX(DriveTrainConstants.leftSlaveTalonID);
    private WPI_TalonSRX rightFollow = new WPI_TalonSRX(DriveTrainConstants.rightSlaveTalonID);

    private DifferentialDrive differentialDrive;
    private boolean invertedDrive = false;

    public DriveTrain() {
        WPI_TalonSRX motors[] = new WPI_TalonSRX[] { leftMaster, rightMaster, leftFollow, rightFollow };
        for (WPI_TalonSRX motor : motors)
            configureMotor(motor);
        differentialDrive = new DifferentialDrive(leftMaster, rightMaster);
    }

    public void periodic(Joystick leftDriver, Joystick rightDriver){
        if(leftDriver.getRawButtonReleased(DriveTrainConstants.invertDriveButton))
            invertedDrive = !invertedDrive;

        if(!invertedDrive)
            differentialDrive.tankDrive(leftDriver.getY(), rightDriver.getY());
        else
            differentialDrive.tankDrive(-rightDriver.getY(), -rightDriver.getY());

        leftFollow.set(ControlMode.Follower, DriveTrainConstants.leftMasterTalonID);
        rightFollow.set(ControlMode.Follower, DriveTrainConstants.rightMasterTalonID);
    }

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