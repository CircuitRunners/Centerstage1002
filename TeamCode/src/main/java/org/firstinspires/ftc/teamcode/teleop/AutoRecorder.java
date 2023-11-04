package org.firstinspires.ftc.teamcode.teleop;


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

@TeleOp (name="Project Neutronium")
public class AutoRecorder extends CommandOpMode {
    private DcMotorEx frontLeft, backLeft, frontRight, backRight, intakeMotor;
    private double frontLeftPower, backLeftPower, frontRightPower, backRightPower;
    private Lift lift;
    private ServoImplEx airplaneLauncher;
    private IMU imu;
    @Override
    public void initialize(){
        schedule(new BulkCacheCommand(hardwareMap));

        lift = new Lift(hardwareMap);

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

//        DcMotorEx[] allMotors =


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


package edu.spa.ftclib.internal;

        import android.os.Environment;

        import java.io.File;
        import java.io.FileWriter;
        import java.io.IOException;
        import java.io.Writer;

/**
 * Created by Gabriel on 2018-01-03.
 * A simple way to log data to a file.
 */

public class Log {
    private static final String BASE_FOLDER_NAME = "FIRST";
    private Writer fileWriter;
    private String line;
    private boolean logTime;
    private long startTime;
    private boolean disabled = false;
    Log(String filename, boolean logTime) {
        if (logTime) startTime = System.nanoTime();
        this.logTime = logTime;
        String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+BASE_FOLDER_NAME;
        File directory = new File(directoryPath);
        //noinspection ResultOfMethodCallIgnored
        directory.mkdir();
        try {
            fileWriter = new FileWriter(directoryPath+"/"+filename+".csv");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public boolean isDisabled() {
        return disabled;
    }

    public void setDisabled(boolean disabled) {
        this.disabled = disabled;
    }

    public void close() {
        try {
            fileWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void update() {
        if (disabled) return;
        try {
            if (logTime) {
                long timeDifference = System.nanoTime()-startTime;
                line = timeDifference/1E9+","+line;
            }
            fileWriter.write(line+"\n");
            line = "";
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void addData(String data) {
        if (disabled) return;
        if (!line.equals("")) line += ",";
        line += data;
    }
    public void addData(Object data) {
        addData(data.toString());
    }
    public void addData(boolean data) {
        addData(String.valueOf(data));
    }
    public void addData(byte data) {
        addData(String.valueOf(data));
    }
    public void addData(char data) {
        addData(String.valueOf(data));
    }
    public void addData(short data) {
        addData(String.valueOf(data));
    }
    public void addData(int data) {
        addData(String.valueOf(data));
    }
    public void addData(long data) {
        addData(String.valueOf(data));
    }
    public void addData(float data) {
        addData(String.valueOf(data));
    }
    public void addData(double data) {
        addData(String.valueOf(data));
    }
}