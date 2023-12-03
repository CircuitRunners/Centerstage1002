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
    ServoModule mod1, mod2, mod3;
    @Override
    public void initialize() {
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx manipulator = new GamepadEx(gamepad2);
        mod1 = new ServoModule(hardwareMap, "leftArm", driver); // 0.83 down 0.37 up 0.75 transport
//        mod2 = new ServoModule(hardwareMap, "rightArm", manipulator);
        mod3 = new ServoModule(hardwareMap, "claw", driver);
    }
    @Override
    public void run() {
        super.run();

//        if (gamepad1.cross) {
//            mod1.mode(false);
//        } else if (gamepad1.circle) {
//            mod1.mode(true);
//        }
//
//        if (gamepad2.cross) {
//            mod2.mode(false);
//        } else if (gamepad2.circle) {
//            mod2.mode(true);
//        }

//        if (gamepad1.dpad_left) {
//            mod3.position1();
//        } else if (gamepad1.dpad_right) {
//            mod3.position2();
//        }
//
//        if (gamepad1.square) {
//            mod3.goToPos1();
//        } else if (gamepad1.circle) {
//            mod3.goToPos2();
//        }

        pseudoPeriodic("leftArm", mod1);
        telemetry.addData("Pos1", mod1.getPos1());
        telemetry.addData("Pos2", mod1.getPos2());

        telemetry.update();
    }

    private void pseudoPeriodic (String tabulate, ServoModule mod) {
        mod.periodic();
        telemetry.addData(tabulate+"\n", mod.getState());
    }

}