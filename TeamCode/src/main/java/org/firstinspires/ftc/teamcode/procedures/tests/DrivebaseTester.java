package org.firstinspires.ftc.teamcode.procedures.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.controllers.subsytems.Drivebase;

@TeleOp (name = "Drivebase Tester")
public class DrivebaseTester extends CommandOpMode {
    public Drivebase drive;
    @Override
    public void initialize () {
        drive = new Drivebase(hardwareMap);
    }

    @Override
    public void run() {
        super.run();

        /* FLIP LEFT 45 DEGREES for correct front left etc
          ^
        <   >
          (down)
         */
        drive.frontLeft.setPower(0);
        drive.backLeft.setPower(0);
        drive.frontRight.setPower(0);
        drive.backRight.setPower(0);

        if (gamepad1.dpad_up) {
            drive.frontLeft.setPower(1);
        }
        if (gamepad1.dpad_right) {
            drive.frontRight.setPower(1);
        }
        if (gamepad1.dpad_left) {
            drive.backLeft.setPower(1);
        }
        if (gamepad1.dpad_down) {
            drive.backRight.setPower(1);
        }
    }

}
