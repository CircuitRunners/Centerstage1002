package org.firstinspires.ftc.teamcode.teleop;


import static org.firstinspires.ftc.teamcode.utilities.CrossBindings.circle;
import static org.firstinspires.ftc.teamcode.utilities.Utilities.debounce;
import static org.firstinspires.ftc.teamcode.utilities.Utilities.differential;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.commands.presets.MoveToScoringCommandEx;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;



import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    public static double DESIRED_DISTANCE = 6;

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = 0;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
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
    private Pivot pivot;

    private ManualLiftCommand manualLiftCommand;
    private IntakeCommandEx intakeCommand;
    private DistanceSensor distanceSensor;
    private ManualLiftResetCommand manualLiftResetCommand;

    private ToggleButtonReader clawReader;

    private boolean isUp = false, isDown = true, isTransport = false, isPressed = false;
    private double upOffset = 0.0, downOffset = 0.0, transportOffset = 0.0;
    private double overallOffset = 0.05;

    public AHRS navx_device;

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
        pivot = new Pivot(hardwareMap);

        manualLiftCommand = new ManualLiftCommand(lift, arm, manipulator);
        manualLiftResetCommand = new ManualLiftResetCommand(lift, manipulator);
        intakeCommand = new IntakeCommandEx(hardwareMap, claw, intake, Intake.IntakePowers.FAST);

        lift.setDefaultCommand(new PerpetualCommand(manualLiftCommand));

//        clawReader = new ToggleButtonReader(manipulator, GamepadKeys.Button.RIGHT_BUMPER);


        new Trigger(() -> manipulator.getLeftY() > 0.4)
                .whenActive(new MoveToScoringCommandEx(lift, arm, claw, MoveToScoringCommandEx.Presets.MID, pivot)
                        .withTimeout(1900)
                        .interruptOn(() -> manualLiftCommand.isManualActive()));

        new Trigger(() -> manipulator.getLeftY() < -0.4)
                .whenActive(new RetractOuttakeCommand(lift, arm, claw, pivot)
                        .withTimeout(1900)
                        .interruptOn(() -> manualLiftCommand.isManualActive()));

        //Mid preset
        new Trigger(() -> manipulator.getLeftX() > 0.6)
                .whenActive(new MoveToScoringCommandEx(lift, arm, claw, MoveToScoringCommandEx.Presets.SHORT, pivot)
                        .withTimeout(1900)
                        .interruptOn(() -> manualLiftCommand.isManualActive()));

        //Short preset
        new Trigger(() -> manipulator.getLeftX() < -0.6)
                .whenActive(new MoveToScoringCommandEx(lift, arm, claw, MoveToScoringCommandEx.Presets.HIGH, pivot)
                        .withTimeout(1900)
                        .interruptOn(() -> manualLiftCommand.isManualActive()));

        // pivoting
        new Trigger(() -> manipulator.getRightY() < -0.4)
                .whenActive(new InstantCommand(()-> {
                        if (!lift.atLowerLimit()){
                            pivot.center();
                        }
                }));
        new Trigger(() -> manipulator.getRightX() > 0.4)
                .whenActive(new InstantCommand(()-> {
                    if (arm.getLeftPosition() > 0.5) {
                        if (!gamepad2.right_stick_button) {
                            pivot.right();
                        } else {
                            // go 180
                            pivot.rightEx();
                        }
                    }
                }));
        new Trigger(() -> manipulator.getRightX() < -0.4)
                .whenActive(new InstantCommand(()-> {
                    if (arm.getLeftPosition() > 0.5) {
                        if (!gamepad2.right_stick_button) {
                            pivot.left();
                        } else {
                            // go 180
                            pivot.leftEx();
                        }
                    }
                }));



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

    public void initAprilTag(HardwareMap hardwareMap) {
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

    }

    @SuppressLint("DefaultLocale")
    @Override
    public void run() {
        super.run();

        boolean targetFound = false;
//        initAprilTag(hardwareMap);

//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        for (AprilTagDetection detection : currentDetections) {
//            if ((detection.metadata != null) &&
//                    ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID))  ){
//                targetFound = true;
//                desiredTag = detection;
//                break;  // don't look any further.
//            } else {
//                telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
//            }
//        }
//
//        if (targetFound) {
//            telemetry.addData(">","Slowing robot down\n");
//            telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
//            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
//            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
//            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
//            drivebase.drive(gamepad1.left_stick_y / 2.0, gamepad1.left_stick_x/2.0, gamepad1.right_stick_x/3.0, gamepad1.cross);
//        } else {
//            telemetry.addData(">","Drive using joysticks to find valid target\n");
//            drivebase.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.cross);
//        }
//        telemetry.update();

        drivebase.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.cross);

        setManualExposure(6, 250);

        telemetry.addData("YAW", drivebase.getCorrectedYaw());
        telemetry.addData("current", intakeCommand.getIntakeCurrent());

         // IN BETA TEST THISIOWHADHSOD

        // Reset heading with MATH
        if (gamepad1.square) {
            drivebase.resetHeading();
            gamepad1.rumble(50);
        }

        telemetry.addLine(String.valueOf(navx_device.getYaw()));
        telemetry.addData("Lift", lift.getLiftPosition());

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

//        // Transfer/Claw
        if (gamepad2.right_bumper) {
            claw.close();
        } else if (gamepad2.left_bumper) {
            claw.open();
        }
        if (gamepad2.left_bumper) {
            if (differential(claw.getPosition() - Claw.IntakePositions.OPEN.position, 0.001)) {
                claw.close();
            } else {
                claw.open();
            }
        }

//        if (clawReader.getState()) {
//            claw.open();
//        } else {
//            claw.close();
//        }

        // Arm commands
        if (debounce(gamepad2.right_trigger)) { // outtake
            pivot.center();
            arm.up();
        } else if (debounce(gamepad2.left_trigger)) { //intake
            pivot.center();
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

        telemetry.addData("Distance Bottom", intakeCommand.distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Distance Top", intakeCommand.distanceSensorTop.getDistance(DistanceUnit.CM));
        telemetry.addData("Claw Status", (claw.getPosition() < 0.45) ? "Open": "Closed");
        telemetry.addData("Arm Position", arm.getLeftPosition());


        if (intakeCommand.distanceSensorTop.getDistance(DistanceUnit.CM) < 4 && intakeCommand.distanceSensor.getDistance(DistanceUnit.CM) < 5 && claw.getPosition() < 0.45 && arm.getLeftPosition() < 0.3) {
            gamepad2.rumble(200);
            gamepad1.rumble(200);
        }



        // Ensure telemetry actually works
        telemetry.update();
    }

    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}