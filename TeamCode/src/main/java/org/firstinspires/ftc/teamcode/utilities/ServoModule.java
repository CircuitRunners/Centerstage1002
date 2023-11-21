package org.firstinspires.ftc.teamcode.utilities;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class ServoModule {
    private ServoImplEx thisServo;

    private double stepValue = 0.1;
    private double pos = 0;

    public ServoModule (HardwareMap hardwareMap, String deviceName, GamepadEx gamepad, double initialPos) {
        this(hardwareMap, deviceName, gamepad);
        pos = initialPos;
    }

    public ServoModule (HardwareMap hardwareMap, String deviceName, GamepadEx gamepad) {
        thisServo = hardwareMap.get(ServoImplEx.class, deviceName);

        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                (()->{ stepValue /= 10; })
        );
        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                (()->{ stepValue *= 10; })
        );

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                (()->{ if (pos <= 1 - stepValue) pos += stepValue; })
        );
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                (()->{ if (pos >= 0 + stepValue) pos -= stepValue; })
        );

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                (()->{ if (pos <= 1 - (10 * stepValue)) pos += 10*stepValue; })
        );
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                (()->{ if (pos >= 0 + 10*stepValue) pos -= 10*stepValue; })
        );
    }

    public String getState () {
        return String.format("POS %f\nSTEP %f", pos, stepValue);
    }

    public void periodic() {
        thisServo.setPosition(pos);
    }
}