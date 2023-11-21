package org.firstinspires.ftc.teamcode.utilities;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;


@TeleOp (name="ServoScanner")
public class ServoScanner extends CommandOpMode {
    ServoModule mod1, mod2;
    @Override
    public void initialize() {
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx manipulator = new GamepadEx(gamepad2);
        mod1 = new ServoModule(hardwareMap, "leftArm", driver);
        mod2 = new ServoModule(hardwareMap, "rightArm", manipulator);
    }
    @Override
    public void run() {
        super.run();

        mod1.periodic();
        mod2.periodic();

        telemetry.addData("mod1\n", mod1.getState());
        telemetry.addData("mod2\n", mod2.getState());


        telemetry.update();
    }

}