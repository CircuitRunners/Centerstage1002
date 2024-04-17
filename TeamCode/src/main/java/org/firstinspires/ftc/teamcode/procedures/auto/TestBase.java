package org.firstinspires.ftc.teamcode.procedures.auto;

import static org.firstinspires.ftc.teamcode.controllers.Constants.AutoPoses.RED_STAGE.r_s_startPos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.controllers.RobotCore;
import org.firstinspires.ftc.teamcode.controllers.auto.pedrocommands.FollowPath;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.PropLocation;

// Complete! :) [who needs I&R anyways?]
@Autonomous (name="mapper")
@Config
public class TestBase extends CommandOpMode {
    private RobotCore daddy;
    private Follower drive;

    PropLocation locationID = PropLocation.RIGHT; // set to right by default

    private PathChain toPixel, toBoard, toStage, toBP;

    Pose2d startPose = r_s_startPos;

    public PathChain directPath(Pose2d startPath, Pose2d endPath) {
        return drive.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(startPath), new Point(endPath))))
                .setLinearHeadingInterpolation(startPath.getHeading(), endPath.getHeading())
                .build();
    }

    public PathChain backwardsPath (Pose2d startPath, Pose2d endPath) {
        return directPath(endPath, startPath);
    }

    @Override
    public void initialize(){
        schedule(new BulkCacheCommand(hardwareMap));
//        detector = new TeamPropDetector(hardwareMap, true, Team.RED);
        daddy = new RobotCore(hardwareMap);
        drive = new Follower(hardwareMap);

        drive.setStartingPose(startPose);

        while(opModeInInit()){
            telemetry.addData("Status", "In Init. Loading...");
            telemetry.update();
//            sleep(50);
        }

        telemetry.addLine("made it opast init");
        telemetry.update();


        schedule(
                new SequentialCommandGroup(
                        new FollowPath(drive, directPath(startPose, new Pose2d(startPose.getX()+0.001, startPose.getY()+0.001, startPose.getHeading()))),
                        new WaitCommand(800)
                )
        );
    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();
        drive.update();

        telemetry.addData("Save Money", drive.getPose());

        telemetry.update();
    }
}