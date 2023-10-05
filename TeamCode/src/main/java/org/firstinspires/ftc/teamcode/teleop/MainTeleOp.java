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
    private double intakePower;
    private double frontLeftPower;
    private double backLeftPower;
    private double frontRightPower;
    private double backRightPower;
    private DcMotorEx frontLeft, backLeft, frontRight, backRight;
    DcMotorEx leftLiftMotor, rightLiftMotor;
    DcMotorEx intakeMotor;


    @Override
    public void initialize() {
        leftLiftMotor = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLiftMotor = hardwareMap.get(DcMotorEx.class, "rightLift");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeftWheel");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeftWheel");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRightWheel");
        backRight = hardwareMap.get(DcMotorEx.class, "backRightWheel");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

    }
    @Override
    public void run() {
        super.run();
        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double liftUp = gamepad2.left_trigger;
        double liftDown = gamepad2.right_trigger;
        double rx = gamepad1.right_stick_x;
        boolean intake = gamepad2.right_bumper;
        boolean reverseIntake = gamepad2.left_bumper;

        if (liftUp != 0) {
            liftPower = liftUp;
        }
        if (liftDown != 0) {
            liftPower = liftDown;
        }

        if (intake){
            intakePower = 1;
        }
        else if (reverseIntake){
            intakePower = -1;
        }
        else{
            intakePower = 0;
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
        intakeMotor.setPower(intakePower);

    }

}
