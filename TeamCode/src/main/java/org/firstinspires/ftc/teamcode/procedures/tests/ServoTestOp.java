package org.firstinspires.ftc.teamcode.procedures.tests;


import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controllers.subsytems.AirplaneLauncher;
import org.firstinspires.ftc.teamcode.controllers.subsytems.ExtendoArm;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.BulkCacheCommand;

@Config
@TeleOp (name="ST OpMode")
public class ServoTestOp extends CommandOpMode {
    private AirplaneLauncher airplaneLauncher;
    public static double pos_down_left = 0;
    public static double pos_down_right = 0;
    ExtendoArm extendo;

    @Override
    public void initialize(){
        // Use a bulk cache to loop faster using old values instead of blocking a thread kinda
        schedule(new BulkCacheCommand(hardwareMap));

        // Subsystems
        extendo = new ExtendoArm(hardwareMap);
    }


    @SuppressLint("DefaultLocale")
    @Override
    public void run() {
        super.run();

        extendo.setPosition(pos_down_left, pos_down_right);

        // Ensure telemetry actually works
        telemetry.update();
    }
}