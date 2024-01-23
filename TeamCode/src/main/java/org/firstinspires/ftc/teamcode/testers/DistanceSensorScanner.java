package org.firstinspires.ftc.teamcode.testers;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utilities.ServoModule;

@TeleOp (name="Distance Sensor scanner Scanner")
public class DistanceSensorScanner extends CommandOpMode {
    DistanceSensor distanceSensor;

    @Override
    public void initialize() {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
    }
    @Override
    public void run() {
        super.run();

        telemetry.addData("Distance, CM", distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.update();
    }

}