package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.util.Color;

class WheelSpinnerConstants {

    // Define Color Mapping (can be offset to account for robot position)
    public static final Color blueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    public static final Color greenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    public static final Color redTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    public static final Color yellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

    public static Map<Character, Color> colorMap;
    static {
        colorMap = new HashMap<>();
        colorMap.put('C', blueTarget);
        colorMap.put('G', greenTarget);
        colorMap.put('R', yellowTarget);
        colorMap.put('Y', yellowTarget);
    }

    // Motor Configuration (Talon IDs)
    public static final int wheelTalonID = 9;
    public static final int hingeTalonID = 11;

    // Button Mappings
    // TODO: UPDATE Mappings
    public static final int spinRotationsButton = 2;
    public static final int cancelRotationButton = 3;
    public static final int rotateToColorButton = 4;

    // Rotation Constants
    // TODO: UPDATE Constants
    public static final int numRotations = 2;
    public static final double wheelSpeed = 0.5;
}

enum WheelSpinnerState {
    WAITING, COUNT_COLOR, MOVE_TO_COLOR
}

public class WheelSpinner {
    private ColorSensorV3 colorSensor;
    private ColorMatch colorMatcher;
    private TalonSRX wheel;
    private TalonSRX hinge;
    private WheelSpinnerState state;

    private Color goalColor;
    private Color initialColor;
    private Color prevColor;
    private int revolutions;

    public WheelSpinner() {
        colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
        colorMatcher = new ColorMatch();

        // Setup Color Matching
        Color[] colors = new Color[] { WheelSpinnerConstants.blueTarget, WheelSpinnerConstants.greenTarget,
                WheelSpinnerConstants.redTarget, WheelSpinnerConstants.yellowTarget };
        for (Color color : colors)
            colorMatcher.addColorMatch(color);

        // Define Hardware Configuration
        wheel = new TalonSRX(WheelSpinnerConstants.wheelTalonID);
        wheel.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
        wheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        wheel.setSelectedSensorPosition(0);

        hinge = new TalonSRX(WheelSpinnerConstants.hingeTalonID);
        hinge.setInverted(true);
        hinge.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
        hinge.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);

        // Initialize State
        state = WheelSpinnerState.WAITING;
        initialColor = null;
        revolutions = 0;
    }

    public void periodic(Joystick mechJoystick) {
        // Get Goal Color Value
        String gameData = DriverStation.getInstance().getGameSpecificMessage();
        if (gameData.length() > 0 && WheelSpinnerConstants.colorMap.containsKey(gameData.charAt(0)))
            goalColor = WheelSpinnerConstants.colorMap.get(gameData.charAt(0));

        // Rotate Set # of Rotations
        if (state == WheelSpinnerState.WAITING
                && mechJoystick.getRawButtonReleased(WheelSpinnerConstants.spinRotationsButton)) {
            initialColor = colorMatcher.matchClosestColor(colorSensor.getColor()).color;
            prevColor = initialColor;
            state = WheelSpinnerState.COUNT_COLOR;
            wheel.set(ControlMode.PercentOutput, WheelSpinnerConstants.wheelSpeed);
        }
        if (state == WheelSpinnerState.COUNT_COLOR) {
            Color currentColor = colorMatcher.matchClosestColor(colorSensor.getColor()).color;
            if (currentColor != prevColor) {
                prevColor = currentColor;
                if (currentColor == initialColor)
                    revolutions++;
            }
            if (revolutions >= WheelSpinnerConstants.numRotations) {
                revolutions = 0;
                wheel.set(ControlMode.PercentOutput, 0);
                state = WheelSpinnerState.WAITING;
            }
        }

        // Rotate to Color State Machine
        if (state == WheelSpinnerState.WAITING
                && mechJoystick.getRawButtonReleased(WheelSpinnerConstants.rotateToColorButton)) {
            state = WheelSpinnerState.MOVE_TO_COLOR;
            wheel.set(ControlMode.PercentOutput, WheelSpinnerConstants.wheelSpeed);
        }
        if (state == WheelSpinnerState.MOVE_TO_COLOR) {
            Color currentColor = colorMatcher.matchClosestColor(colorSensor.getColor()).color;
            if (currentColor == goalColor) {
                wheel.set(ControlMode.PercentOutput, 0);
                state = WheelSpinnerState.WAITING;
            }
        }

        // Universal Cancel Button
        if (mechJoystick.getRawButtonReleased(WheelSpinnerConstants.cancelRotationButton)) {
            revolutions = 0;
            wheel.set(ControlMode.PercentOutput, 0);
            state = WheelSpinnerState.WAITING;
        }
    }
}