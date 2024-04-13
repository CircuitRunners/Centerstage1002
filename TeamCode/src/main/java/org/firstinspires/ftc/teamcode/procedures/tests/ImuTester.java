package org.firstinspires.ftc.teamcode.procedures.tests;


import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Drivebase;


@TeleOp (name="ImuTester")
public class ImuTester extends CommandOpMode {
    private DcMotorEx frontLeft, backLeft, frontRight, backRight, intakeMotor;
    private DcMotorEx[] drivebaseMotors;
    private double frontLeftPower, backLeftPower, frontRightPower, backRightPower;

    private IMU imu;
    private static IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                    RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
            )
    );;

    private Drivebase drivebase;

    private boolean isUp = false, isDown = true, isTransport = false, isPressed = false;
    private double upOffset = 0.0, downOffset = 0.0, transportOffset = 0.0;
    private double overallOffset = 0.05;

    private AHRS navx_device;

    @Override
    public void initialize(){
        // Use a bulk cache to loop faster using old values instead of blocking a thread kinda

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        schedule(new BulkCacheCommand(hardwareMap));
//        PhotonCore.start(hardwareMap);

        navx_device = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navX2"), AHRS.DeviceDataType.kProcessedData);

        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx manipulator = new GamepadEx(gamepad2);

        drivebase = new Drivebase(hardwareMap);

//        lift.setDefaultCommand(new PerpetualCommand(manualLiftCommand));
//
//        new Trigger(() -> manipulator.getLeftY() > 0.4)
//                .whenActive(new MoveToScoringCommand(lift, arm, transfer, MoveToScoringCommand.Presets.HIGH)
//                        .withTimeout(1900)
//                        .interruptOn(() -> manualLiftCommand.isManualActive()));
//
//        new Trigger(() -> manipulator.getLeftY() < -0.6)
//                .whenActive(new RetractOuttakeCommand(lift, arm, transfer)
//                        .withTimeout(1900)
//                        .interruptOn(() -> manualLiftCommand.isManualActive()));
//
//        //Mid preset
//        new Trigger(() -> manipulator.getRightY() > -0.4)
//                .whenActive(new MoveToScoringCommand(lift, arm, transfer, MoveToScoringCommand.Presets.MID)
//                        .withTimeout(1900)
//                        .interruptOn(() -> manualLiftCommand.isManualActive()));
//
//        //Short preset
//        new Trigger(() -> manipulator.getRightY() < 0.4)
//                .whenActive(new MoveToScoringCommand(lift, arm, transfer, MoveToScoringCommand.Presets.SHORT)
//                        .withTimeout(1900)
//                        .interruptOn(() -> manualLiftCommand.isManualActive()));
//
//        manipulator.getGamepadButton(GamepadKeys.Button.Y) // Playstation Triangle
//                .whenHeld(manualLiftResetCommand);
    }


    @SuppressLint("DefaultLocale")
    @Override
    public void run() {
        super.run();

        drivebase.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, false);

        // Reset heading with MATH
        if (gamepad1.square) {
            drivebase.resetHeading();
            gamepad1.rumble(50);
        }

        // Lift brakes when not doing anything

//        double lift_speed = 0.7;
//        double gravity_constant = 0.23;
//        if (gamepad2.dpad_up) {
//            lift.setLiftPower(-lift_speed);
//        } else if (gamepad2.dpad_down) {
//            lift.setLiftPower(1-gravity_constant);
//        } else {
//            lift.brake_power();
//        }
//
//        double keepRobotUpPowerWinch = 0.2;
//        if (debounce(gamepad2.right_stick_y)) {
//            lift.hangPower(gamepad2.right_stick_y);
//        } else if (debounce(gamepad2.right_stick_x)) {
//            lift.hangPower(-keepRobotUpPowerWinch);
//        } else {
//            lift.hangPower(0);
//        }
//
//
//        // Intake Assembly
//        intake.setPower(gamepad1.right_trigger, gamepad1.left_trigger);
//
//        // Airplane Launcher
////        airplaneLauncher.processInput(gamepad2.cross, false);
//
//        // Transfer/Claw
//        if (gamepad2.right_bumper) {
//            transfer.close();
//        } else if (gamepad2.left_bumper) {
//            transfer.open();
//        }
//
//        // Arm commands
//        if (debounce(gamepad2.right_trigger)) { // outtake
//            arm.up();
//        } else if (debounce(gamepad2.left_trigger)) { //intake
//            arm.down();
//        }
//
//        // Front "Extendo" Arm up/down
//        if (gamepad1.left_bumper){
//            frontArm.up();
//        } else if(gamepad1.right_bumper){
//            frontArm.down();
//        }
//
//        if (gamepad2.circle){
//            lift.enableHang();
//        } else if (gamepad2.square) {
//            lift.initialInitHang();
//        }

        // Ensure telemetry actually works
        telemetry.addData("IMU Yaw", navx_device.getYaw());
        telemetry.addData("IMU Yaw 2", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        telemetry.update();
    }
}