package org.firstinspires.ftc.teamcode.procedures.tests;


import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controllers.common.utilities.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.controllers.subsytems.AirplaneLauncher;

@Disabled
@TeleOp (name="AirTest")
public class AirplaneTestTeleOp extends CommandOpMode {
    private AirplaneLauncher airplaneLauncher;

    @Override
    public void initialize(){
        // Use a bulk cache to loop faster using old values instead of blocking a thread kinda
        schedule(new BulkCacheCommand(hardwareMap));

        GamepadEx manipulator = new GamepadEx(gamepad2);

        // Subsystems
        airplaneLauncher = new AirplaneLauncher(hardwareMap);
    }


    @SuppressLint("DefaultLocale")
    @Override
    public void run() {
        super.run();

        // Airplane Launcher
        airplaneLauncher.processInput(gamepad2.cross, false);

        // Ensure telemetry actually works
        telemetry.update();
    }
}