package org.firstinspires.ftc.teamcode.procedures.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.controllers.auto.pedrocommands.FollowPath;
import org.firstinspires.ftc.teamcode.controllers.auto.pedrocommands.HoldPoint;
import org.firstinspires.ftc.teamcode.controllers.auto.pedrocommands.WaitForReachedTValue;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.PropLocation;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.Team;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Arm;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Claw;
import org.firstinspires.ftc.teamcode.controllers.subsytems.ExtendoArm;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Intake;

import org.firstinspires.ftc.teamcode.controllers.subsytems.Lift;
import org.firstinspires.ftc.teamcode.controllers.vision.TeamPropDetector;

// Complete! :) [who needs I&R anyways?]
@Autonomous (name="%% BlueClose25")
@Config

public class PedroBase extends CommandOpMode {
    private Follower drive;

//    TeamPropDetector detector;

    public static Pose2d startingPose = new Pose2d(15.5, 61.75, Math.toRadians(270));

    PropLocation locationID = PropLocation.RIGHT; // set to right by default

    private PathChain middleSpikeMark, middleBackboardTraj, middleStackTraj, middleBackboardStackTraj, middleStack2Traj, middleBackboardStack2Traj, middleStack3Traj, middleBackboardStack3Traj, middleEndTraj;

    private Lift lift;
    private Arm arm;
    private Claw claw;
    private ExtendoArm extendo;

    private Intake intake;

    double center_line_y = -12.5, stack_x = -55.5, avoidance_x_constant = 1,
            fastVelocity = 60, offsetFromBoard = 4.0;

    Pose2d startPose = new Pose2d(10.5, -62.5, Math.toRadians(90.00));

    Pose2d pixel_left = new Pose2d(22.4, -42, Math.toRadians(90.00));
    Pose2d pixel_center = new Pose2d(11.84, -33.90, Math.toRadians(90.00));
    Pose2d pixel_right = new Pose2d(27, -40, Math.toRadians(105));


    // TODO set the x values to the correct on
    Pose2d boardPosition_left = new Pose2d(52.65, -27 - offsetFromBoard, Math.toRadians(0));
    Pose2d boardPosition_center = new Pose2d(52.65, -35 - offsetFromBoard, Math.toRadians(0));
    Pose2d boardPosition_right = new Pose2d(52.65, -38 - offsetFromBoard, Math.toRadians(0));

    ParallelCommandGroup scheduledCommandGroup;


    @Override
    public void initialize(){
        schedule(new BulkCacheCommand(hardwareMap));
//        detector = new TeamPropDetector(hardwareMap, true, Team.RED);

        drive = new Follower(hardwareMap);

        intake = new Intake(hardwareMap);

        lift = new Lift(hardwareMap);
        claw = new Claw(hardwareMap);
        arm = new Arm(hardwareMap);
        extendo = new ExtendoArm(hardwareMap);

        lift.initialInitHang();

        drive.setStartingPose(startingPose);

        middleSpikeMark = drive.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(startingPose), new Point(12, 37.5, Point.CARTESIAN))))
                .setLinearHeadingInterpolation(startingPose.getHeading(), Math.toRadians(220))
                .build();

        middleBackboardTraj = drive.pathBuilder()
                .addPath(new Path(new BezierLine(middleSpikeMark.getPath(0).getLastControlPoint(), new Point(52.5, 27, Point.CARTESIAN))))
                .setLinearHeadingInterpolation(Math.toRadians(220), Math.PI, 0.7)
                .build();

        middleStackTraj = drive.pathBuilder()
                .addPath(new BezierCurve(
                        middleBackboardTraj.getPath(0).getLastControlPoint(),
                        new Point(30, 10.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .addPath(new BezierLine(
                        new Point(30, 10.5, Point.CARTESIAN),
                        new Point(-45, 11.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .build();

        middleBackboardStackTraj = drive.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(-40, 10.5, Point.CARTESIAN),
                        new Point(37, 10.5, Point.CARTESIAN),
                        new Point(52.2, 28, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .build();
        middleStack2Traj = drive.pathBuilder()
                .addPath(new BezierCurve(
                        middleBackboardTraj.getPath(0).getLastControlPoint(),
                        new Point(30, 10.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .addPath(new BezierLine(
                        new Point(30, 10.5, Point.CARTESIAN),
                        new Point(-45, 11.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .build();

        middleBackboardStack2Traj = drive.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(-40, 10.5, Point.CARTESIAN),
                        new Point(37, 10.5, Point.CARTESIAN),
                        new Point(52.2, 28, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .build();
        middleStack3Traj = drive.pathBuilder()
                .addPath(new BezierCurve(
                        middleBackboardTraj.getPath(0).getLastControlPoint(),
                        new Point(30, 10.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .addPath(new BezierLine(
                        new Point(30, 10.5, Point.CARTESIAN),
                        new Point(-65, 11.75, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .build();

        middleBackboardStack3Traj = drive.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(-40, 10.5, Point.CARTESIAN),
                        new Point(37, 10.5, Point.CARTESIAN),
                        new Point(52.2, 28, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .build();
        middleEndTraj = drive.pathBuilder()
                .addPath(new Path(new BezierLine(middleBackboardStackTraj.getPath(0).getLastControlPoint(), new Point(47, 35, Point.CARTESIAN))))
                .setConstantHeadingInterpolation(Math.PI)
                .build();

        while(opModeInInit()){
            telemetry.addData("Status", "In Init. Loading...");
            telemetry.update();
//            sleep(50);
        }

        telemetry.addLine("made it opast init");
        telemetry.update();

        schedule(
                new SequentialCommandGroup(
                        new WaitCommand(200),
                        new FollowPath(drive, middleSpikeMark),
                        new WaitCommand(200)
                )
        );
    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();
        drive.update();

        telemetry.update();
    }
}