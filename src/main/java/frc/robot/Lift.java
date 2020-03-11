package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;

class LiftConstants {
    // Talon IDs
    public static final int leadScrewTalonID = 7;
    public static final int pivotTalonID = 6;

    // Button Mapping
    // TODO: Update to Operator Driver Preferences
    public static final int leadScrewUpButton = 4;
    public static final int leadScrewDownButton = 5;
    public static final int pivotUpButton = 6;
    public static final int pivotDownButton = 7;

    // Rotation Speeds
    public static final double leadScrewSpeed = 0.5;
    public static final double pivotSpeed = 0.5;
}

public class Lift {
    private TalonSRX leadScrew = new TalonSRX(LiftConstants.leadScrewTalonID);
    private TalonSRX pivot = new TalonSRX(LiftConstants.pivotTalonID);

    public Lift() {
        // Lead Screw Configuration
        leadScrew.setInverted(true);
        leadScrew.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
        leadScrew.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);

        // Pivot Configuration
        pivot.setInverted(true);
        pivot.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
        pivot.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    }

    public void periodic(Joystick mechJoystick) {
        // Lead Screw Control
        if (mechJoystick.getRawButtonPressed(LiftConstants.leadScrewUpButton))
            leadScrew.set(ControlMode.PercentOutput, LiftConstants.leadScrewSpeed);
        else if (mechJoystick.getRawButtonPressed(LiftConstants.leadScrewDownButton))
            leadScrew.set(ControlMode.PercentOutput, -LiftConstants.leadScrewSpeed);

        // Lift Pivot Control
        if (mechJoystick.getRawButtonPressed(LiftConstants.pivotUpButton))
            pivot.set(ControlMode.PercentOutput, LiftConstants.pivotSpeed);
        else if (mechJoystick.getRawButtonPressed(LiftConstants.pivotDownButton))
            pivot.set(ControlMode.PercentOutput, -LiftConstants.pivotSpeed);
    }
}