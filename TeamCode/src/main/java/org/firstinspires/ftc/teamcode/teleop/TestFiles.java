package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

@Disabled
@TeleOp(name="Logger v0.5.1")
public class TestFiles extends CommandOpMode {
    private List<String> logData;
    private boolean isRecording = false;

    private long recordingStartTime;
    private boolean recordingFinished;
    private boolean writeInProgress;
    private static final String BASE_FOLDER_NAME = "FIRST";

    // TeleOp Declarations
    private String directoryPath;

    @Override
    public void initialize() {
        File file = new File("/sdcard/file124.txt");

        logData = new ArrayList<>();

        directoryPath =  "/sdcard/" + BASE_FOLDER_NAME + "/logData";
//        directoryPath = Environment.getExternalStorageDirectory().getAbsolutePath() + "/" + BASE_FOLDER_NAME + "/logData";

        telemetry.addLine(directoryPath);
        telemetry.update();

        File directory = new File(directoryPath);
        if (!directory.exists()) {
            boolean success = directory.mkdirs();
            if (!success) {
                telemetry.addLine("Failed to create directory: " + directoryPath);
                telemetry.update();
            }
        }

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
            recordingFinished = true;
        }

        if (recordingFinished && !writeInProgress) {
            int runNumber = 1;

            // Check for existing files and increment runNumber if necessary
            File file;
            do {
                telemetry.addLine(directoryPath + runNumber + ".txt");
                telemetry.update();
                file = new File(directoryPath + runNumber + ".txt");
                runNumber++;
            } while (file.exists());
            try (PrintWriter out = new PrintWriter(directoryPath + runNumber + ".txt")) {
                for (String data : logData) {
                    out.println(data);
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
            writeInProgress = true;
        }

        String test_string = "this is a string";
        timestamp = System.currentTimeMillis() - recordingStartTime;
        logData.add(timestamp + "," + test_string);

        teleOpRun();
    }

    public void teleOpInitialize () {
        // Init mechanisms
    }
    public void teleOpRun () {
        super.run();
        // Actually run the code lmao
    }
}