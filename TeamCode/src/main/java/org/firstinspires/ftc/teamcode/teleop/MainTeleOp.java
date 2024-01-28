package org.firstinspires.ftc.teamcode.teleop;


import static org.firstinspires.ftc.teamcode.utilities.CrossBindings.circle;
import static org.firstinspires.ftc.teamcode.utilities.CrossBindings.triangle;
import static org.firstinspires.ftc.teamcode.utilities.Utilities.debounce;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommandEx;
import org.firstinspires.ftc.teamcode.commands.liftcommands.ManualLiftCommand;
import org.firstinspires.ftc.teamcode.commands.liftcommands.ManualLiftResetCommand;
import org.firstinspires.ftc.teamcode.commands.presets.MoveToScoringCommand;
import org.firstinspires.ftc.teamcode.commands.presets.RetractOuttakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.AirplaneLauncher;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.ExtendoArm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.utilities.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@TeleOp (name="MainTeleOp")
public class MainTeleOp extends CommandOpMode {
    private DcMotorEx frontLeft, backLeft, frontRight, backRight, intakeMotor;
    private DcMotorEx[] drivebaseMotors;
    private double frontLeftPower, backLeftPower, frontRightPower, backRightPower;

    private Lift lift;
    private AirplaneLauncher airplaneLauncher;
    private Drivebase drivebase;
    private Arm arm;
    private Claw claw;
    private Intake intake;
    private ExtendoArm frontArm;
    private ManualLiftCommand manualLiftCommand;
    private IntakeCommandEx intakeCommand;
    private DistanceSensor distanceSensor;
    private ManualLiftResetCommand manualLiftResetCommand;

    private boolean isUp = false, isDown = true, isTransport = false, isPressed = false;
    private double upOffset = 0.0, downOffset = 0.0, transportOffset = 0.0;
    private double overallOffset = 0.05;

    private AHRS navx_device;

    @Override
    public void initialize(){
        // Use a bulk cache to loop faster using old values instead of blocking a thread kinda
        schedule(new BulkCacheCommand(hardwareMap));
//        PhotonCore.start(hardwareMap);

        navx_device = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navX2"), AHRS.DeviceDataType.kProcessedData);

        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx manipulator = new GamepadEx(gamepad2);

        // Subsystems
        lift = new Lift(hardwareMap);
        airplaneLauncher = new AirplaneLauncher(hardwareMap);
        drivebase = new Drivebase(hardwareMap);
        arm = new Arm(hardwareMap);
        claw = new Claw(hardwareMap);
        intake = new Intake(hardwareMap);

        manualLiftCommand = new ManualLiftCommand(lift, manipulator);
        manualLiftResetCommand = new ManualLiftResetCommand(lift, manipulator);
        intakeCommand = new IntakeCommandEx(hardwareMap, claw, intake, Intake.IntakePowers.FAST);

        lift.setDefaultCommand(new PerpetualCommand(manualLiftCommand));

        new Trigger(() -> manipulator.getLeftY() > 0.4)
                .whenActive(new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.HIGH)
                        .withTimeout(1900)
                        .interruptOn(() -> manualLiftCommand.isManualActive()));

        new Trigger(() -> manipulator.getLeftY() < -0.6)
                .whenActive(new RetractOuttakeCommand(lift, arm, claw)
                        .withTimeout(1900)
                        .interruptOn(() -> manualLiftCommand.isManualActive()));

        //Mid preset
        new Trigger(() -> manipulator.getRightY() > -0.4)
                .whenActive(new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.MID)
                        .withTimeout(1900)
                        .interruptOn(() -> manualLiftCommand.isManualActive()));

        //Short preset
        new Trigger(() -> manipulator.getRightY() < 0.4)
                .whenActive(new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.SHORT)
                        .withTimeout(1900)
                        .interruptOn(() -> manualLiftCommand.isManualActive()));

        manipulator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT) // Playstation Triangle
                .whenHeld(manualLiftResetCommand);

        driver.getGamepadButton(circle)
                .whenActive(intakeCommand);

        frontArm = new ExtendoArm(hardwareMap);
        claw.open();
        lift.initialInitHang();

        telemetry.addLine("Ready for start!");
        telemetry.update();
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

        telemetry.addLine(String.valueOf(navx_device.getFusedHeading()));

        if (gamepad2.triangle) {

            lift.hangPower(1);
            lift.setLiftPower(-.4);
        } else if (gamepad2.cross) {
            lift.hangPower(-0.5);
            lift.setLiftPower(0.6);
        } else {
            lift.hangPower(0);
        }

        if (gamepad2.dpad_right) {
            lift.hangPower(0.2);
        }

        // Intake Assembly

        if (debounce(gamepad1.left_trigger) || debounce(gamepad1.right_trigger)) {
            if (intakeCommand.isScheduled()) {
                telemetry.addLine("ICSX2");
                intakeCommand.cancel();
            }
            intake.setPower(gamepad1.right_trigger, gamepad1.left_trigger);
        } else if (intakeCommand.isScheduled()){
            telemetry.addLine("Intake Command Scheduled");
        } else {
            intake.setPower(0);
        }

        // Airplane Launcher
        airplaneLauncher.processInput(gamepad1.dpad_down, false);

        // Transfer/Claw
        if (gamepad2.right_bumper) {
            claw.close();
        } else if (gamepad2.left_bumper) {
            claw.open();
        }

        // Arm commands
        if (debounce(gamepad2.right_trigger)) { // outtake
            arm.up();
        } else if (debounce(gamepad2.left_trigger)) { //intake
            arm.down();
        }

        // Front "Extendo" Arm up/down
        if (gamepad1.left_bumper){
            frontArm.up();
        } else if(gamepad1.right_bumper){
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