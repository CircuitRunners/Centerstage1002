package org.firstinspires.ftc.teamcode.teleop;

import android.os.Environment;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Repertoire Recorder")
public class TeleOpRecorder extends CommandOpMode {
    private List<String> gamepadData;
    private boolean isRecording = false;
    private String previousGamepad1, previousGamepad2;
    private long recordingStartTime;

    // TeleOp Declarations

    private DcMotorEx frontLeft, backLeft, frontRight, backRight, intakeMotor;
    private double frontLeftPower, backLeftPower, frontRightPower, backRightPower;
    private Lift lift;
    private ServoImplEx airplaneLauncher;
    private IMU imu;

    // End TeleOp Declarations

    @Override
    public void initialize() {
        schedule(new BulkCacheCommand(hardwareMap));

        gamepadData = new ArrayList<>();
        previousGamepad1 = "";
        previousGamepad2 = "";

        // The rest of your initialization code...
        teleOpInitialize();
    }

    @Override
    public void run() {
        super.run();
        long timestamp;
        // Check if the left d-pad is pressed to start recording
        if (gamepad1.dpad_left && !isRecording) {
            isRecording = true;
            recordingStartTime = System.currentTimeMillis();
        }

        // Check if the right d-pad is pressed to stop recording
        if (gamepad1.dpad_right && isRecording) {
            isRecording = false;
        }

        // Save the recording to the file only if there's a difference
        if (isRecording){
            timestamp = System.currentTimeMillis() - recordingStartTime;
            telemetry.addLine(String.valueOf(timestamp));
            telemetry.update();

            String gamepad1hash = gamepad1.toString();
            String gamepad2hash = gamepad2.toString();

            if (!gamepad1hash.equals(previousGamepad1) || !gamepad2hash.equals(previousGamepad2)) {
                gamepadData.add(timestamp + "," + gamepad1hash + "," + gamepad2hash);
                previousGamepad1 = gamepad1hash;
                previousGamepad2 = gamepad2hash;
            }
        }

        // The rest of your run() code...
        teleOpRun();
    }

    @Override
    public void reset() {
        int runNumber = 1;
        // Check for existing files and increment runNumber if necessary
        File file;
        do {
            file = new File(Environment.getExternalStorageDirectory().getPath()+"/FIRST/gamepadData" + runNumber + ".txt");
            runNumber++;
        } while (file.exists());
        // Write the gamepad data to a file when the OpMode is stopped
        try (PrintWriter out = new PrintWriter(Environment.getExternalStorageDirectory().getPath()+"/FIRST/gamepadData.txt")) {
            for (String data : gamepadData) {
                out.println(data);
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    public void teleOpInitialize () {
        lift = new Lift(hardwareMap);

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");


        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        airplaneLauncher = hardwareMap.get(ServoImplEx.class, "airplaneLauncher");

        imu = hardwareMap.get(IMU.class, "imu");

        airplaneLauncher.setPosition(0.72);

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(parameters);
    }
    public void teleOpRun () {
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

        if (Math.abs(gamepad2.right_trigger) > 0.05) {
            // Outtake
            intakeMotor.setPower(-gamepad2.right_trigger * (3.0/4.0));
        } else if (Math.abs(gamepad2.left_trigger) > 0.05) {
            // Intake
            intakeMotor.setPower(gamepad2.left_trigger * (3.0/4.0));
        } else {
            intakeMotor.setPower(0);
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