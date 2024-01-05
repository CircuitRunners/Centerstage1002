package org.firstinspires.ftc.teamcode.teleop;


import static org.firstinspires.ftc.teamcode.utilities.Utilities.debounce;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.commands.liftcommands.ManualLiftCommand;
import org.firstinspires.ftc.teamcode.commands.liftcommands.ManualLiftResetCommand;
import org.firstinspires.ftc.teamcode.subsystems.AirplaneLauncher;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.ExtendoArm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.utilities.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
@TeleOp (name="MainTeleOp")
public class MainTeleOp extends CommandOpMode {
    private DcMotorEx frontLeft, backLeft, frontRight, backRight, intakeMotor;
    private DcMotorEx[] drivebaseMotors;
    private double frontLeftPower, backLeftPower, frontRightPower, backRightPower;

    private Lift lift;
//    private AirplaneLauncher airplaneLauncher;
    private Drivebase drivebase;
    private Arm arm;
    private Transfer transfer;
    private Intake intake;
    private ExtendoArm frontArm;
//    private AHRS altHeadRefSys;

    private ServoImplEx rightArm, leftArm, claw;
    private IMU imu;

    private ManualLiftCommand manualLiftCommand;
    private ManualLiftResetCommand manualLiftResetCommand;

    private boolean isUp = false, isDown = true, isTransport = false, isPressed = false;
    private double upOffset = 0.0, downOffset = 0.0, transportOffset = 0.0;
    private double overallOffset = 0.05;

    @Override
    public void initialize(){
        // Use a bulk cache to loop faster using old values instead of blocking a thread kinda
        schedule(new BulkCacheCommand(hardwareMap));

        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx manipulator = new GamepadEx(gamepad2);

        // Subsystems
        lift = new Lift(hardwareMap);
//        airplaneLauncher = new AirplaneLauncher(hardwareMap);
        drivebase = new Drivebase(hardwareMap);
        arm = new Arm(hardwareMap);
        transfer = new Transfer(hardwareMap);
        intake = new Intake(hardwareMap);

        manualLiftCommand = new ManualLiftCommand(lift, manipulator);
        manualLiftResetCommand = new ManualLiftResetCommand(lift, manipulator);

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
        frontArm = new ExtendoArm(hardwareMap);
    }


    @SuppressLint("DefaultLocale")
    @Override
    public void run() {
        super.run();

        drivebase.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        // Reset heading with MATH
        if (gamepad1.square) {
            drivebase.resetHeading();
            gamepad1.rumble(50);
        }


        // Lift brakes when not doing anything
        lift.brake_power();

        if (debounce(gamepad2.left_stick_y)) {
            lift.setLiftPower(gamepad2.left_stick_y);
        }

        if (debounce(gamepad2.right_trigger)) {
            lift.hangPower(gamepad2.right_trigger);
        } else if (debounce(gamepad2.left_trigger)) {
            lift.hangPower(-0.2);
        } else {
            lift.hangPower(0);
        }


        // Intake Assembly
        intake.setPower(gamepad1.left_trigger, gamepad1.right_trigger);

        // Airplane Launcher
//        airplaneLauncher.processInput(gamepad2.cross, false);

        // Transfer/Claw
        if (gamepad1.right_bumper) {
            transfer.close();
        } else if (gamepad1.left_bumper) {
            transfer.open();
        }

        // Arm commands
        if (gamepad2.right_bumper) { // outtake
            arm.up();
        } else if (gamepad2.left_bumper) { //intake
            arm.down();
        }

        // Front "Extendo" Arm up/down
        if (gamepad2.triangle){
            frontArm.up();
        } else if(gamepad2.cross){
            frontArm.down();
        }
        if (gamepad2.circle){
            lift.enableHang();
        } else if (gamepad2.square) {
            lift.initialInitHang();
        }


        // Ensure telemetry actually works
        telemetry.update();
    }
}