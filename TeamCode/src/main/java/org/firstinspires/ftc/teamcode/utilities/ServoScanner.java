package org.firstinspires.ftc.teamcode.utilities;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;


@TeleOp (name="ServoScanner")
public class ServoScanner extends CommandOpMode {

    private String deviceName = "rightArm";
    private double stepValue = 0.1;
    private double pos = 0;

    ServoImplEx thisCrazyServo;

    @Override
    public void initialize() {
        thisCrazyServo = hardwareMap.get(ServoImplEx.class, deviceName);

        thisCrazyServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        GamepadEx driver = new GamepadEx(gamepad1);

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                (()->{ stepValue /= 10; })
        );
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                (()->{ stepValue *= 10; })
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                (()->{ if (pos <= 1 - stepValue) pos += stepValue; })
        );
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                (()->{ if (pos >= 0 + stepValue) pos -= stepValue; })
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                (()->{ if (pos <= 1 - (10 * stepValue)) pos += 10*stepValue; })
        );
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                (()->{ if (pos >= 0 + 10*stepValue) pos -= 10*stepValue; })
        );

    }
    @Override
    public void run() {
        super.run();

        if ((Math.abs(gamepad1.left_stick_y) > 0.01) && gamepad1.square) pos = gamepad1.left_stick_y;

        thisCrazyServo.setPosition(pos);

        telemetry.addData("Position", thisCrazyServo.getPosition());
        telemetry.addData("Step Value", stepValue);
        telemetry.addData("Step Value", gamepad1.left_stick_y);

        telemetry.update();
    }

}