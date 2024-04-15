package org.firstinspires.ftc.teamcode.controllers;


import com.arcrobotics.ftclib.command.Robot;
import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.controllers.subsytems.AirplaneLauncher;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Arm;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Claw;
import org.firstinspires.ftc.teamcode.controllers.subsytems.ExtendoArm;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Intake;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Lift;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Pivot;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Sensors;

public class RobotCore extends Robot {
    public Lift lift;
    public AirplaneLauncher airplaneLauncher;
//    public Drivebase drivebase;
    public Arm arm;
    public Claw claw;
    public Intake intake;
    public ExtendoArm frontArm;
    public Pivot pivot;
    public Sensors sensors;

    public Follower drive;

    public AHRS navx_device;

    public RobotCore (HardwareMap hardwareMap) {

        // Schedule to clear cache continuously (manual mode)
        schedule(new BulkCacheCommand(hardwareMap));

        // Auto drive mechanics
        drive = new Follower(hardwareMap);

        // Initialize subsystems
        lift = new Lift(hardwareMap);
        airplaneLauncher = new AirplaneLauncher(hardwareMap);
//        drivebase = new Drivebase(hardwareMap);
        arm = new Arm(hardwareMap);
        claw = new Claw(hardwareMap);
        intake = new Intake(hardwareMap);
        pivot = new Pivot(hardwareMap);
        sensors = new Sensors(hardwareMap);

        navx_device = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navX2"), AHRS.DeviceDataType.kProcessedData);

        frontArm = new ExtendoArm(hardwareMap);
    }
}