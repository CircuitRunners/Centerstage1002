package org.firstinspires.ftc.teamcode.procedures.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.controllers.RobotCore;
import org.firstinspires.ftc.teamcode.controllers.auto.pedrocommands.FollowPath;
import org.firstinspires.ftc.teamcode.controllers.auto.pedrocommands.HoldPoint;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.PropLocation;

// Complete! :) [who needs I&R anyways?]
@Autonomous (name="redstace 2 0")
@Config
public class PedroBase extends CommandOpMode {
    private RobotCore daddy;
    private Follower drive;

    PropLocation locationID = PropLocation.RIGHT; // set to right by default

    private PathChain toPixel, toBoard, toStage, toBP;

    Pose2d startPose = new Pose2d(15.75, -63, Math.toRadians(90.00));
    Pose2d backPoint = new Pose2d(Math.toRadians(90.00));
    Pose2d pixelCenter = new Pose2d(11.84, -33.90, Math.toRadians(110.00));

    Pose2d boardPosCenter = new Pose2d(50.33, -36, Math.toRadians(0));
    Pose2d stagePos = new Pose2d(50.33, -14, Math.toRadians(-90));
    Pose2d parkPos = new Pose2d(62.75, -14, Math.toRadians(-90));

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

        toPixel = directPath(startPose, pixelCenter);

        toBoard = drive.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(27, -54),
                        new Point(pixelCenter)
                )).setConstantHeadingInterpolation(Math.toDegrees(110))
                .addPath(new BezierCurve(
                        new Point(27, -54),
                        new Point(boardPosCenter)
                )).setLinearHeadingInterpolation(Math.toDegrees(110), boardPosCenter.getHeading())
                .build();

        toStage = directPath(boardPosCenter, stagePos);
        toBP = directPath(stagePos, parkPos);

        while(opModeInInit()){
            telemetry.addData("Status", "In Init. Loading...");
            telemetry.update();
//            sleep(50);
        }

        telemetry.addLine("made it opast init");
        telemetry.update();


        schedule(
                new SequentialCommandGroup(
                        new FollowPath(drive, toPixel),
                        new FollowPath(drive, toBoard),
                        new HoldPoint(drive, boardPosCenter),
                        new WaitCommand(1500),
                        new FollowPath(drive, toStage),
                        new FollowPath(drive, toBP)
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