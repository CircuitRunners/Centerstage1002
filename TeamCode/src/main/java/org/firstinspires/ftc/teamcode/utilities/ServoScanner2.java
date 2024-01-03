package org.firstinspires.ftc.teamcode.utilities;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp (name="ServoScanner 2")
public class ServoScanner2 extends CommandOpMode {
    ServoModule mod1, mod2, mod3;
    @Override
    public void initialize() {
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx manipulator = new GamepadEx(gamepad2);

//        DOWN(0.61, 0.82), // 0.61 0.82
//        SCORING(0.9, 0.333); // 0.9 0.333

//        mod1 = new ServoModule(hardwareMap, "rightArm", driver);
        mod2 = new ServoModule(hardwareMap, "leftArm", manipulator);
//        mod3 = new ServoModule(hardwareMap, "claw", driver);
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

//        pseudoPeriodic("rightArm test", mod1);

//        mod1.mode(false);

        pseudoPeriodic(" test", mod2);
//        pseudoPeriodic("claw", mod3);
        telemetry.update();
    }

    private void pseudoPeriodic (String tabulate, ServoModule mod) {
        mod.periodic();
        telemetry.addData(tabulate+"\n", mod.getState());
    }

}