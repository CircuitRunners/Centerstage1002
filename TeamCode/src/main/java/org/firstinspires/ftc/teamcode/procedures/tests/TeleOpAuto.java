package org.firstinspires.ftc.teamcode.procedures.tests;

import android.os.Environment;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.PseudoGamepad;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Lift;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

@Disabled
@Autonomous(name="Repertoire Auto")
public class TeleOpAuto extends CommandOpMode {
    private long startTime;
    private List<GamepadState> gamepadData;

    // TeleOp Declarations

    private DcMotorEx frontLeft, backLeft, frontRight, backRight, intakeMotor;
    private double frontLeftPower, backLeftPower, frontRightPower, backRightPower;
    private Lift lift;
    private ServoImplEx airplaneLauncher;
    private IMU imu;

    // End TeleOp Declarations

    @Override
    public void initialize() {
        startTime = 0;
        int autoToRun = 1; // This needs to be 1 or greater
        gamepadData = loadRecording(autoToRun);

        teleOpInitialize();
        // The rest of your initialization code...
    }

    @Override
    public void run() {
        super.run();
        if (startTime == 0) {
            startTime = System.currentTimeMillis();
        }

        String line;
        long timeIntoAuto = System.currentTimeMillis() - startTime;

        for (GamepadState state : gamepadData) {
            // Wait until the timestamp to run, otherwise skip
            // timestamp is ms into auto from the recorder
            while (true) {
                if (timeIntoAuto >= state.timestamp) break;
            }

            runTeleOpCode(state.pseudoGamepad1, state.pseudoGamepad2);
        }
    }

    public List<GamepadState> loadRecording (int recordingNumber) {
        BufferedReader gamepadReader;
        ArrayList<GamepadState> gamepadStates = new ArrayList();
        try {
            gamepadReader = new BufferedReader(new FileReader(String.format("%s/FIRST/gamepadData%d.txt", Environment.getExternalStorageDirectory().getPath(), recordingNumber)));

            String line;
            while ((line = gamepadReader.readLine()) != null) {
                // Parse the timestamp and gamepad states from the line
                // gamepadData.add(timestamp + "," + gamepad1hash + "," + gamepad2hash); <-- the format for the read lines
                String[] parts = line.split(",");

                long timestamp = Long.parseLong(parts[0]);

                PseudoGamepad gamepad1State = PseudoGamepad.parse(parts[1]);
                PseudoGamepad gamepad2State = PseudoGamepad.parse(parts[2]);

                // Create a list for this line and add it to the main list
                GamepadState thisState = new GamepadState(timestamp, gamepad1State, gamepad2State);
                gamepadStates.add(thisState);
            }

            gamepadReader.close();
        } catch (IOException e) {
            telemetry.addLine(String.valueOf(e));
            telemetry.update();
        }

        return gamepadStates;
    }

    public void runTeleOpCode (PseudoGamepad gamepad1, PseudoGamepad gamepad2) {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.06; // change 1.06 to counter bad strafing
        double rx = gamepad1.right_stick_x;

        if (gamepad1.x) {
            imu.resetYaw();
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
}

// A gamepad state is where the controllers/gamepads are at a specific point in time
class GamepadState {
    public long timestamp;
    public PseudoGamepad pseudoGamepad1;
    public PseudoGamepad pseudoGamepad2;

    public GamepadState (long thisTimestamp, PseudoGamepad thisGamepad1, PseudoGamepad thisGamepad2) {
        timestamp = thisTimestamp;
        pseudoGamepad1 = thisGamepad1;
        pseudoGamepad2 = thisGamepad2;
    }
}