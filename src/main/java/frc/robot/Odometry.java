package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

public class Odometry {
    private AHRS gyro;
    private DifferentialDriveOdometry odometry;
    private WPI_TalonSRX leftMaster, rightMaster;

    public Odometry(WPI_TalonSRX leftMaster, WPI_TalonSRX rightMaster) {
        gyro = new AHRS(SPI.Port.kMXP);
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(gyro.getYaw()));
    }

    public void teleopPeriodic() {
        // Update the odometry information using our encoders
        // TODO: unit conversions for ticks to meters
        odometry.update(Rotation2d.fromDegrees(gyro.getYaw()), leftMaster.getSelectedSensorPosition(),
                rightMaster.getSelectedSensorPosition());

    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftMaster.getSelectedSensorVelocity(),
                rightMaster.getSelectedSensorVelocity());
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
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        gyro.reset();
    }
}