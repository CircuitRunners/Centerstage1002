package org.firstinspires.ftc.teamcode.procedures;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.controllers.VisionRobotCore;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.PropLocation;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.Team;

@Config
public abstract class AutoBase extends CommandOpMode {
    public VisionRobotCore robot;

    PropLocation locationID = PropLocation.MIDDLE; // set to right by default
    Pose2d startPose = new Pose2d(0, 0,  Math.toRadians(90.00));

    Team team = Team.RED;

    @Override
    public void initialize(){
        robot = new VisionRobotCore(hardwareMap, team);

        robot.drive.setPoseEstimate(startPose);
        robot.lift.initialInitHang();

        build();

//        robot.detector.startStream();
        while(opModeInInit()){
//            locationID = robot.detector.update();
            telemetry.addData("Status", "In Init. Loading...");
            telemetry.addData("Prop", locationID.getLocation());
            telemetry.update();

//            sleep(30);
        }
//        robot.detector.stopStream();

        telemetry.addData("Status", "Loaded!");
        telemetry.update();

//        runAuto(locationID);
        runAuto();
    }

    public void setStartPose(Pose2d startPose) {
        this.startPose = startPose;
    }

    public void setTeam(Team team) {
        this.team = team;
    }

//    @Override
//    public void run() {
//        CommandScheduler.getInstance().run();
//
//        telemetry.update();
//    }

    public abstract void build();

//    public abstract void runAuto(PropLocation location);
    public abstract void runAuto();
}