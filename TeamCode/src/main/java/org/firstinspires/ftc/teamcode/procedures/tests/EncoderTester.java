package org.firstinspires.ftc.teamcode.procedures.tests;


import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.controllers.common.util.Encoder;


@TeleOp (name="Encoder Tester")
public class EncoderTester extends CommandOpMode {
    private Encoder leftEncoder, rightEncoder, frontEncoder;
    private double initialFrontEncoder, initialRightEncoder, initialLeftEncoder;

    @Override
    public void initialize(){
        // Use a bulk cache to loop faster using old values instead of blocking a thread kinda

        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontRight"));
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightLift"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backLeft"));

        initialFrontEncoder = frontEncoder.getCurrentPosition();
        initialLeftEncoder = leftEncoder.getCurrentPosition();
        initialRightEncoder = rightEncoder.getCurrentPosition();

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        frontEncoder.setDirection(Encoder.Direction.REVERSE);
//        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        leftEncoder.setDirection(Encoder.Direction.REVERSE);

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
        telemetry.addData("left encoder", leftEncoder.getCurrentPosition() - initialLeftEncoder);
        telemetry.addData("right encoder", rightEncoder.getCurrentPosition() - initialRightEncoder);
        telemetry.addData("front encoder", frontEncoder.getCurrentPosition() - initialFrontEncoder);

        telemetry.update();
    }
}