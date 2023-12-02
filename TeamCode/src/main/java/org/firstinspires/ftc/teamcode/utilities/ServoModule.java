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

    private double pos1, pos2;

    public ServoModule (HardwareMap hardwareMap, String deviceName, GamepadEx gamepad, double initialPos) {
        this(hardwareMap, deviceName, gamepad);
        pos = initialPos;
    }

    public void mode (boolean on) {
        if (on) {
            thisServo.setPwmEnable();
        }
        else if (!on){
            thisServo.setPwmDisable();
        }
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

//        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
//                (()->{ if (pos <= 1 - (10 * stepValue)) pos += 10*stepValue; })
//        );
//        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
//                (()->{ if (pos >= 0 + 10*stepValue) pos -= 10*stepValue; })
//        );
    }

    public String getState () {
        return String.format("POS %f\nSTEP %f", pos, stepValue);
    }

    public void position1 (double p1) {
        pos1 = p1;
    }

    public void position1 () {
        pos1 = pos;
    }

    public void position2 () {
        pos2 = pos;
    }

    public void position2 (double p2) {
        pos2 = p2;
    }

    public void goToPos1 () {
        pos = pos1;
    }

    public void goToPos2 () {
        pos = pos2;
    }

    public double getPos () {
        return pos;
    }

    public double getPos1 () {
        return pos1;
    }
    public double getPos2 () {
        return pos2;
    }


    public void periodic() {
        thisServo.setPosition(pos);
    }
}