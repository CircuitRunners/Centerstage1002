package org.firstinspires.ftc.teamcode.procedures.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controllers.common.utilities.ServoModule;

@Disabled
@TeleOp (name="Claw Scanner")
public class ClawScanner extends CommandOpMode {
    ServoModule mod1;
    GamepadEx driver, manipulator;
    String servoToTest = "claw";

    @Override
    public void initialize() {
        driver = new GamepadEx(gamepad1);
        manipulator = new GamepadEx(gamepad2);

        mod1 = new ServoModule(hardwareMap, servoToTest, driver);
    }
    @Override
    public void run() {
        super.run();

        pseudoPeriodic(servoToTest, mod1);

        if(gamepad2.y){
            telemetry.addData("Servo off", servoToTest);
            mod1.mode(false);
        } else if(gamepad2.a){
            telemetry.addData("Servo on", servoToTest);
            mod1.mode(true);
        }

        telemetry.update();
    }

    private void pseudoPeriodic (String tabulate, ServoModule mod) {
        mod.periodic();
        telemetry.addData(tabulate+"\n", mod.getState());
    }

}