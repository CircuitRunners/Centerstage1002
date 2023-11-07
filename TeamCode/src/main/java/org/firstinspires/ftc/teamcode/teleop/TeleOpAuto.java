package org.firstinspires.ftc.teamcode.teleop;

import android.os.Environment;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

@Autonomous(name="Repertoire Auto")
public class TeleOpAuto extends CommandOpMode {
    private BufferedReader gamepadReader;
    private long startTime;
    private List<List<Object>> gamepadData;

    @Override
    public void initialize() {
        startTime = 0;
        gamepadData = new ArrayList<>();

        // Initialize the BufferedReader
        try {
            gamepadReader = new BufferedReader(new FileReader(Environment.getExternalStorageDirectory().getPath()+String.format("/FIRST/gamepadData%s.txt","1")));

            String line;
            while ((line = gamepadReader.readLine()) != null) {
                // Parse the timestamp and gamepad states from the line
                String[] parts = line.split(",");
                long timestamp = Long.parseLong(parts[0]);

                PseudoGamepad gamepad1State = PseudoGamepad.parse(parts[1]);
                PseudoGamepad gamepad2State = PseudoGamepad.parse(parts[2]);

                // Create a list for this line and add it to the main list
                List<Object> lineData = new ArrayList<>();
                lineData.add(timestamp);
                lineData.add(gamepad1State);
                lineData.add(gamepad2State);
                gamepadData.add(lineData);
            }
        } catch (IOException e) {
            telemetry.addLine(String.valueOf(e));
            telemetry.update();
        }

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

        try {


            while ((line = gamepadReader.readLine()) != null) {
                // Parse the timestamp and gamepad states from the line
                String[] parts = line.split(",");
                long timestamp = Long.parseLong(parts[0]);

                PseudoGamepad gamepad1State = PseudoGamepad.parse(parts[1]);
                PseudoGamepad gamepad2State = PseudoGamepad.parse(parts[2]);



                // Wait until it's time to replay this gamepad state
                while (timeIntoAuto < timestamp) {
                    Thread.sleep(1);
                }

                // The rest of your run() code...
            }
        } catch (IOException | InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void reset() {
        // Close the reader when the OpMode is stopped
        try {
            gamepadReader.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
