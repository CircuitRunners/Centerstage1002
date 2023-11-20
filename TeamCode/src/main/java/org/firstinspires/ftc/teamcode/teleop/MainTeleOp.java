package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commands.liftcommands.ManualLiftCommand;
import org.firstinspires.ftc.teamcode.commands.liftcommands.ManualLiftResetCommand;
import org.firstinspires.ftc.teamcode.subsystems.AirplaneLauncher;
import org.firstinspires.ftc.teamcode.utilities.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.subsystems.Lift;



@TeleOp (name="MainTeleOp")
public class MainTeleOp extends CommandOpMode{
    private DcMotorEx frontLeft, backLeft, frontRight, backRight, intakeMotor;
    private DcMotorEx[] drivebaseMotors;
    private double frontLeftPower, backLeftPower, frontRightPower, backRightPower;
    private Lift lift;

    private AirplaneLauncher airplaneLauncher;

    private ServoImplEx rightArm, leftArm, claw;
    private IMU imu;

    private ManualLiftCommand manualLiftCommand;
    private ManualLiftResetCommand manualLiftResetCommand;

    @Override
    public void initialize(){
        // Use a bulk cache to loop faster using old values instead of blocking a thread kinda
        schedule(new BulkCacheCommand(hardwareMap));

        // Subsystems
        lift = new Lift(hardwareMap);
        airplaneLauncher = new AirplaneLauncher(hardwareMap);

        // Vanilla initializations
        imu = hardwareMap.get(IMU.class, "imu");

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        drivebaseMotors = new DcMotorEx[] {frontLeft, backLeft, frontRight, backRight};

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

        // Modifications of Vanilla
        for (DcMotorEx motor : drivebaseMotors) {
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);


        leftArm = hardwareMap.get(ServoImplEx.class, "leftArm");
        rightArm = hardwareMap.get(ServoImplEx.class, "rightArm");
        claw = hardwareMap.get(ServoImplEx.class, "claw");



        airplaneLauncher.setPosition(0.58);

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
        double x = gamepad1.left_stick_x * 1.06; //TODO change 1.06 to counter bad strafing
        double rx = gamepad1.right_stick_x;

        // Reset heading with MATH
        if (gamepad1.x) {
            // reset stuff here
            gamepad1.rumble(100);
        }

        // Make lift not do stupid :)
        lift.setLiftPower(0);

        telemetry.addData("Lift Position", lift.getLiftPosition());

        // Lift
        if (Math.abs(gamepad2.left_stick_y) > 0.05) { // <-- Debounce
            lift.setLiftPower(gamepad2.left_stick_y);
        }

        // Intake Assembly
        if (Math.abs(gamepad1.right_trigger) > 0.05) {
            // Outtake
            intakeMotor.setPower(-gamepad1.right_trigger);
        } else if (Math.abs(gamepad1.left_trigger) > 0.05) {
            // Intake
            intakeMotor.setPower(gamepad1.left_trigger);
        } else {
            intakeMotor.setPower(0);
        }

//        if (Math.abs(gamepad2.right_stick_y) > 0.05) { //debounce
//            airplaneLauncher.setPosition(Math.abs(gamepad2.right_stick_y));
//        }

//        telemetry.addData("AirplaneL Position", airplaneLauncher.getPosition());

        // Airplane Launcher
//        if (gamepad2.dpad_up) {
//            airplaneLauncher.setPosition(0.58);
//        }
        if (gamepad2.a) {
            airplaneLauncher.setPosition(0.50);
        }

        // Claw
        if (gamepad2.dpad_right) {
            claw.setPosition(.54);
        }
        else if (gamepad2.dpad_left) {
            claw.setPosition(.44);
        }

        // right arm .448 up
        // right .9045 down
        // .54 claw open
        // .44 claw closed
        // left arm up .488
        // left arm .9499

        telemetry.addLine("right " + String.valueOf(gamepad2.right_trigger));
        telemetry.addLine("left "+String.valueOf(gamepad2.left_trigger));

        if (Math.abs(gamepad2.right_trigger) > 0.05) {
            // Outtake
            rightArm.setPosition(gamepad2.right_trigger);
        }
        if (Math.abs(gamepad2.left_trigger) > 0.05) {
            // Outtake
            leftArm.setPosition(gamepad2.left_trigger);
        }

        // Arm commands
        if (gamepad2.right_bumper) {
            leftArm.setPosition(.42); //.488
            rightArm.setPosition(.0); // CHANGED
        } else if (gamepad2.left_bumper) {
            leftArm.setPosition(.84);
            rightArm.setPosition(.5);
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        Vector2d vec = new Vector2d(x, y).rotated(-botHeading);

        x = vec.getX();
        y = vec.getY();

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        denominator = 1;

        frontLeftPower = (y + x + rx) / denominator; // forward is positive
        backLeftPower = (y - x + rx) / denominator; // positive
        frontRightPower = (y - x - rx) / denominator; // forward is negative
        backRightPower = (y + x - rx) / denominator; // negative

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        telemetry.update();
    }
}