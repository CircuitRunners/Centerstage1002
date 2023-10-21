package org.firstinspires.ftc.teamcode.teleop;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@TeleOp (name="King Richard IV")
public class PushbotFieldCentric extends CommandOpMode{
    private DcMotorEx frontLeft, backLeft, frontRight, backRight;
    private double frontLeftPower, backLeftPower, frontRightPower, backRightPower;
    private Lift lift;
    private ServoImplEx airplaneLauncher;
    private IMU imu;
    @Override
    public void initialize(){
        lift = new Lift(hardwareMap);

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");


        airplaneLauncher = hardwareMap.get(ServoImplEx.class, "airplaneLauncher");

        airplaneLauncher.setPosition(0.72);

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(parameters);
    }


    @Override
    public void run() {
        super.run();
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.06; // change 1.06 to counter bad strafing
        double rx = gamepad1.right_stick_x;

        if (gamepad1.x) {
            imu.resetYaw();
            gamepad1.rumble(100);
        }

        lift.setLiftPower(0);

        telemetry.addData("Lift Position", lift.getLiftPosition());

        if (Math.abs(gamepad2.left_stick_y) > 0.05) {
            lift.setLiftPower(gamepad2.left_stick_y);
        }

        if (Math.abs(gamepad2.right_stick_y) > 0.05) { //debounce
            airplaneLauncher.setPosition(Math.abs(gamepad2.right_stick_y));
        }

        telemetry.addData("AirplaneL Position", airplaneLauncher.getPosition());

        if (gamepad2.dpad_up){
            airplaneLauncher.setPosition(0.72);
        }
        else if (gamepad2.dpad_down) {
            airplaneLauncher.setPosition(0.18);
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        Vector2d vec = new Vector2d(x, y).rotated(-botHeading);

        x = vec.getX();
        y = vec.getY();

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        frontLeftPower = -(y + x + rx) / denominator; // forward is positive
        backLeftPower = -(y - x + rx) / denominator; // positive

        frontRightPower = (y - x - rx) / denominator; // forward is negative
        backRightPower = (y + x - rx) / denominator; // negative

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        telemetry.update();
    }

}
