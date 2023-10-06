package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class MainTeleOp extends CommandOpMode {

    private double liftPower;
    private double frontLeftPower;
    private double backLeftPower;
    private double frontRightPower;
    private double backRightPower;
    private DcMotorEx frontLeft, backLeft, frontRight, backRight;

    DcMotorEx leftLiftMotor, rightLiftMotor;


    @Override
    public void initialize() {
        leftLiftMotor = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLiftMotor = hardwareMap.get(DcMotorEx.class, "rightLift");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeftWheel");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeftWheel");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRightWheel");
        backRight = hardwareMap.get(DcMotorEx.class, "backRightWheel");

    }
    @Override
    public void run() {
        super.run();
        private double y = gamepad1.left_stick_y;
        private double x = gamepad1.left_stick_x;
        private double liftUp = gamepad2.left_trigger;
        private double liftDown = gamepad2.right_trigger;
        private double rx = gamepad1.right_stick;
        private double x2 = gamepad2.left_stick_x;
        if (liftUp != 0) {
            liftPower = liftUp;
        }
        if (liftDown != 0) {
            liftPower = liftDown;
        }
        frontLeftPower = Range.clip(-(y-x), -1.0, 1.0);
        backLeftPower = Range.clip(-(y-x), -1.0, 1.0);
        frontRightPower = Range.clip(y+x, -1.0, 1.0);
        backRightPower = Range.clip(y+x, -1.0, 1.0)
        leftLiftMotor.setPower(liftPower);
        rightLiftMotor.setPower(-liftPower);
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

    }

}
