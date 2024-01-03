package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static com.qualcomm.robotcore.hardware.IMU.Parameters;

public class Drivebase extends SubsystemBase {
    private DcMotorEx frontLeft, backLeft, frontRight, backRight;
    private DcMotorEx[] allDrivebaseMotors;
    private IMU imu;
    private static Parameters parameters = new Parameters(
            new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                    RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
            )
    );;

    // Default constructor
    public Drivebase(HardwareMap hardwareMap) {
        this(hardwareMap, parameters); // Call the constructor with default parameters
    }

    // Constructor with IMU parameters
    public Drivebase(HardwareMap hardwareMap, Parameters parameters) {
        // Drivetrain motors setup
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        allDrivebaseMotors = new DcMotorEx[]{frontLeft, backLeft, frontRight, backRight};

        // We love the IMU..!
        imu = hardwareMap.get(IMU.class, "imu");

        // Begin doing things
        setMotorBehavior(allDrivebaseMotors);
        initializeIMU(parameters); // Initialize IMU with the given parameters
    }

    private void setMotorBehavior (DcMotorEx[] motors) {
        // Set motor directions and zero power behavior
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        // Have all motors brake
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }
    }

    public void initializeIMU (Parameters parameters) {
        imu.initialize(parameters);
    }

    private double preprocessInput(double variable) {
        // Input pre process here
        return variable;
    }

    private double postprocessInput(double variable) {
        // Input postprocess here
        return variable;
    }

    private double transformYInput (double y) {
        y = preprocessInput(y);

        // Actual processing
        // The value of the y joystick is reversed--remember?
        y = -1 * y;

        y = postprocessInput(y);
        return y;
    }

    private double transformXInput (double x) {
        x = preprocessInput(x);

        // Actual processing
        // Counteract bad strafing
        // TODO to change this as we test!
        x = x * 1.06;

        x = postprocessInput(x);
        return x;
    }

    private double transformRotationInput (double rx) {
        rx = preprocessInput(rx);

        // Actual processing
        // This does nothing

        rx = postprocessInput(rx);
        return rx;
    }

    public void drive(double left_stick_y, double left_stick_x, double right_stick_x) {

        double y = transformYInput(left_stick_y); // y && strafe forward back
        double x = transformXInput(left_stick_x); // x && strafe right and left
        double rx = transformRotationInput(right_stick_x); // rx && turn left right angular

        // Calculate the robot's heading from the IMU
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        Vector2d botVector = new Vector2d(x, y).rotated(-botHeading);

        // Apply the calculated heading to the input vector for field centric
        x = botVector.getX(); // strafe r/l transform values
        y = botVector.getY(); // strafe f/b transform values
        // note rx is not here since rotation is always field centric!

        // Calculate the motor powers
        double frontLeftPower = y + x + rx;
        double frontRightPower = y - x - rx;

        double backLeftPower = y - x + rx;
        double backRightPower = y + x - rx;

        // Find the max power and make sure it ain't greater than 1
        double denominator = max(
                max(abs(frontLeftPower), abs(backLeftPower)),
                max(abs(frontRightPower), abs(backRightPower))
        );

        if (denominator > 1.0) {
            frontLeftPower /= denominator;
            backLeftPower /= denominator;
            frontRightPower /= denominator;
            backRightPower /= denominator;
        }

        // Old method below, to ensure the CONTROLLERS no more than 1
        // double denominator = max(abs(y) + abs(x) + abs(rx), 1);

        // Set the motor powers
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    public void resetHeading() {
        // This is the imu inbuilt version, uh you will need to change for new one
        imu.resetYaw();
    }
}