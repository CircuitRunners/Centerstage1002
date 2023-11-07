package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.vision.TeamPropVision;
import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;

@TeleOp(name = "Vision TeleOp", group = "TeleOp")
public class VisionTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the vision system
        TeamPropVision vision = new TeamPropVision(telemetry);

        // Initialize the camera
        VideoCapture cap = new VideoCapture(0);  // Use 0 to access default camera
        cap.set(Videoio.CAP_PROP_FRAME_WIDTH, 320);
        cap.set(Videoio.CAP_PROP_FRAME_HEIGHT, 240);

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            Mat frame = new Mat();
            if (cap.read(frame)) {
                // Process the frame
                vision.processFrame(frame);
            }
            // Other robot code here...
        }

        cap.release();
    }
}