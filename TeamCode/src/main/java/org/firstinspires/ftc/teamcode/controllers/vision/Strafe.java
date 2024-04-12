package org.firstinspires.ftc.teamcode.controllers.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
public class Strafe extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor rearLeft;
    private DcMotor rearRight;

    private final double kP = 0.1;
    private final double kI = 0.0;
    private final double kD = 0.0;
    private double previousError = 0;
    private double integral = 0;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "lf");
        frontRight = hardwareMap.get(DcMotor.class, "lr");
        rearLeft = hardwareMap.get(DcMotor.class, "rb");
        rearRight = hardwareMap.get(DcMotor.class, "rf");

        initAprilTag();

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double error = getBearingError();
            double correction = pidCorrection(error);

            // Using correction for motor powers
            double power = 0.5 + correction;
            frontLeft.setPower(power);
            frontRight.setPower(-power);
            rearLeft.setPower(-power);
            rearRight.setPower(power);
            telemetryAprilTag();

            telemetry.update();

            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }

            sleep(20);


//            if (shouldStopStrafing()) {
//                // Stop the robot
//
//                /* This doesn't actually work properly
//                (we need to implement some form of PID in
//                order to facilitate an accurate stop). The
//                current implementation doesn't account for
//                momentum which will just carry the robot forward.
//                */
//
//                frontLeft.setPower(0);
//                frontRight.setPower(0);
//                rearLeft.setPower(0);
//                rearRight.setPower(0);
//            } else {
//                double power = 0.5;
//                frontLeft.setPower(power);
//                frontRight.setPower(-power);
//                rearLeft.setPower(-power);
//                rearRight.setPower(power);
//            }


            idle();
        }
        visionPortal.close();
    }

    /**
     * Initialize the april tag processor
     * @return
     */
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
    }

    /**
     * Add telemetry about AprilTag detections
     * @return
     */
    private void telemetryAprilTag() {
        List <AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }

        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }

    public boolean shouldStopStrafing() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (Math.abs(detection.ftcPose.bearing) < 5) {
                return true;
            }
        }
        return false;
    }

    private double getBearingError() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            return detection.ftcPose.bearing;
        }
        return 0;
    }

    private double pidCorrection(double error) {
        double derivative = error - previousError;
        integral += error;

        double output = kP * error + kI * integral + kD * derivative;
        previousError = error;
        return output;
    }

}