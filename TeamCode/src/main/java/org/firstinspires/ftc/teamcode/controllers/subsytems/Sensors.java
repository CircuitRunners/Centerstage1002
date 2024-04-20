package org.firstinspires.ftc.teamcode.controllers.subsytems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Sensors extends SubsystemBase {
    private DistanceSensor distanceSensorBottom, distanceSensorTop;

    public Sensors(HardwareMap hardwareMap) {
        distanceSensorBottom = hardwareMap.get(DistanceSensor.class, "topDistanceSensor");
        distanceSensorTop = hardwareMap.get(DistanceSensor.class, "distanceSensor");
    }

    public double getTopDistance() {
        return getDistance(distanceSensorTop);
    }

    public double getBottomDistance() {
        return getDistance(distanceSensorBottom);
    }
    private double getDistance (DistanceSensor distanceSensor) {
        return distanceSensor.getDistance(DistanceUnit.CM);
    }

    private double getDistance (DistanceSensor distanceSensor, DistanceUnit unit) {
        return distanceSensor.getDistance(unit);
    }

}
