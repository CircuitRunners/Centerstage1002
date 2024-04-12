package org.firstinspires.ftc.teamcode.procedures.auto._REMOVE_OLD.backstage;

import static org.firstinspires.ftc.teamcode.controllers.common.utilities.CrossBindings.rotationConstant;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


import org.firstinspires.ftc.teamcode.controllers.auto.pedrocommands.FollowPath;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Arm;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Claw;
import org.firstinspires.ftc.teamcode.controllers.subsytems.ExtendoArm;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Intake;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Lift;
import org.firstinspires.ftc.teamcode.controllers.auto.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.PropLocation;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.Team;
import org.firstinspires.ftc.teamcode.controllers.vision.TeamPropDetector;

// Complete! :) [who needs I&R anyways?]
@Disabled
@Autonomous (name="BLUE BS PEDRO")
public class BlueStagePedro extends CommandOpMode {
    private Follower pfollower;
    private TrajectorySequence ONE_GLOBAL;

    private TrajectorySequence THREE_PIXEL_ON_BACKDROP;
    private PathChain middleSpikeMark, middleBackboardTraj, middleStackTraj, middleBackboardStackTraj, middleStack2Traj, middleBackboardStack2Traj, middleStack3Traj, middleBackboardStack3Traj, middleEndTraj;

    private TeamPropDetector detector;
    private PropLocation locationID = PropLocation.MIDDLE; // set to center by default

    //    private Pose2d startPose = Pose2dMapped(9.00, -61.50, Math.toRadians(90.00));
    private Pose2d startPose = Pose2dMapped(13.80, 63, Math.toRadians(270.00));
    private Lift lift;
    private Arm arm;
    private Claw claw;
    private ExtendoArm extendo;

    private Intake intake;

    public static Pose2d startingPose = new Pose2d(15.5, 61.75, Math.toRadians(270));

    // Path gen - https://www.desmos.com/calculator/3so1zx0hcd
    @Override
    public void initialize(){
        pfollower = new Follower(hardwareMap);
        detector = new TeamPropDetector(hardwareMap, true, Team.BLUE);
        schedule(new BulkCacheCommand(hardwareMap));

        intake = new Intake(hardwareMap);

        lift = new Lift(hardwareMap);
        claw = new Claw(hardwareMap);
        arm = new Arm(hardwareMap);
        extendo = new ExtendoArm(hardwareMap);

        lift.initialInitHang();

        pfollower.setStartingPose(startPose);


        middleSpikeMark = pfollower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(startingPose), new Point(11.5, 35.5, Point.CARTESIAN))))
                .setConstantHeadingInterpolation(startingPose.getHeading())
                .build();

        middleBackboardTraj = pfollower.pathBuilder()
                .addPath(new Path(new BezierLine(middleSpikeMark.getPath(0).getLastControlPoint(), new Point(52, 35, Point.CARTESIAN))))
                .setLinearHeadingInterpolation(startingPose.getHeading(), Math.PI*0.8)
                .build();

        middleStackTraj = pfollower.pathBuilder()
                .addPath(new BezierCurve(
                        middleBackboardTraj.getPath(0).getLastControlPoint(),
                        new Point(30, 10.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                //.addTemporalCallback(1, () -> vision.setProcessorEnabled(stackProcessor, true))
                .addPath(new BezierLine(
                        new Point(30, 10.5, Point.CARTESIAN),
                        new Point(-45, 11.75, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .build();

        middleBackboardStackTraj = pfollower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(-40, 10.5, Point.CARTESIAN),
                        new Point(37, 10.5, Point.CARTESIAN),
                        new Point(51, 28, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
//                .addParametricCallback(0.1, () -> {
//                    robot.m_intake.raise();
//                    robot.m_intake.back();
//                })
//                .addParametricCallback(0.5, () -> {
//                    robot.m_intake.stop();
//                    robot.m_conveyor.stop();
//                    robot.m_outtake.extend(-0.07);
//                    robot.m_lift.setRelativePosition(600);
//                })
                .build();
        middleStack2Traj = pfollower.pathBuilder()
                .addPath(new BezierCurve(
                        middleBackboardTraj.getPath(0).getLastControlPoint(),
                        new Point(30, 10.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                //.addTemporalCallback(1, () -> vision.setProcessorEnabled(stackProcessor, true))
                .addPath(new BezierLine(
                        new Point(30, 10.5, Point.CARTESIAN),
                        new Point(-45, 11.75, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .build();

        middleBackboardStack2Traj = pfollower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(-40, 10.5, Point.CARTESIAN),
                        new Point(37, 10.5, Point.CARTESIAN),
                        new Point(51, 28, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
//                .addParametricCallback(0.1, () -> {
//                    robot.m_intake.raise();
//                    robot.m_intake.back();
//                })
//                .addParametricCallback(0.5, () -> {
//                    robot.m_intake.stop();
//                    robot.m_conveyor.stop();
//                    robot.m_outtake.extend(-0.07);
//                    robot.m_lift.setRelativePosition(900);
//                })
                .build();
        middleStack3Traj = pfollower.pathBuilder()
                .addPath(new BezierCurve(
                        middleBackboardTraj.getPath(0).getLastControlPoint(),
                        new Point(30, 10.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                //.addTemporalCallback(1, () -> vision.setProcessorEnabled(stackProcessor, true))
                .addPath(new BezierLine(
                        new Point(30, 10.5, Point.CARTESIAN),
                        new Point(-65, 11.75, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
//                .addParametricCallback(0.8, () -> {
//                    robot.m_intake.suck();
//                    robot.m_conveyor.up();
//                })
                .build();

        middleBackboardStack3Traj = pfollower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(-40, 10.5, Point.CARTESIAN),
                        new Point(37, 10.5, Point.CARTESIAN),
                        new Point(51, 28, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
//                .addParametricCallback(0.1, () -> {
//                    robot.m_intake.raise();
//                    robot.m_intake.back();
//                })
//                .addParametricCallback(0.5, () -> {
//                    robot.m_intake.stop();
//                    robot.m_conveyor.stop();
//                    robot.m_outtake.extend(-0.07);
//                    robot.m_lift.setRelativePosition(900);
//                })
                .build();
        middleEndTraj = pfollower.pathBuilder()
                .addPath(new Path(new BezierLine(middleBackboardStackTraj.getPath(0).getLastControlPoint(), new Point(47, 35, Point.CARTESIAN))))
                .setConstantHeadingInterpolation(Math.PI)
                .build();

        detector.startStream();

        while(opModeInInit()){
            locationID = detector.update();
            //telemetry.addLine("Ready for start!");
            telemetry.addData("Prop", locationID.getLocation());
            telemetry.update();
        }

        detector.stopStream();

        // Purple pixel
//        switch (locationID) {
//            case LEFT: {
//                ONE_GLOBAL = drive.trajectorySequenceBuilder(startPose)
//                        .lineTo(Vector2dMapped(26.63, 39.57))
//                        .lineTo(Vector2dMapped(26.83, 42.57))
//                        .lineTo(Vector2dMapped(32.30, 45.89))
//                        .build();
//                THREE_PIXEL_ON_BACKDROP = drive.trajectorySequenceBuilder(ONE_GLOBAL.end())
//                        .splineToLinearHeading(Pose2dMapped(48.31, 45.47, Math.toRadians(360.00)), MathtoRadians(-2.5))
//                        .lineTo(Vector2dMapped(52.72, 45.47))
//                        .build();
//                break;
//            }
//            case MIDDLE: {
//                ONE_GLOBAL = drive.trajectorySequenceBuilder(startPose)
//                        .lineTo(Vector2dMapped(12.3800001, 34.24))
//                        .lineTo(Vector2dMapped(32.30, 45.89))
//                        .build();
//                THREE_PIXEL_ON_BACKDROP = drive.trajectorySequenceBuilder(ONE_GLOBAL.end())
//                        .splineToLinearHeading(Pose2dMapped(48.31, 37, Math.toRadians(360.00)), MathtoRadians(-2.5))
//                        .lineTo(Vector2dMapped(52.72, 37))
//                        .build();
//                break;
//            }
//            case RIGHT: {
//                ONE_GLOBAL = drive.trajectorySequenceBuilder(startPose)
//                        .lineToLinearHeading(Pose2dMapped(18.65, 44.64, Math.toRadians(205.00)))
//                        .lineTo(Vector2dMapped(9.95, 39))
//                        .lineToLinearHeading(Pose2dMapped(38.4, 45.89, Math.toRadians(362.5)))
//                        .build();
//                THREE_PIXEL_ON_BACKDROP = drive.trajectorySequenceBuilder(ONE_GLOBAL.end())
//                        .splineToLinearHeading(Pose2dMapped(48.31, 30.97, Math.toRadians(360.00)), MathtoRadians(-2.5))
//                        .lineTo(Vector2dMapped(52.72, 30.97))
//                        .build();
//                break;
//            }
//        }

//        TrajectorySequence THREE_PIXEL_ON_BACKDROP = drive.trajectorySequenceBuilder(ONE_GLOBAL.end())
//                .splineToLinearHeading(Pose2dMapped(48.31, -36.47, Math.toRadians(0.00)), MathtoRadians(0.00))
//                .lineTo(Vector2dMapped(52.22, -36.47))
//                .build();
//
//        // Stack
//        TrajectorySequence FOUR_TO_LIGHTSPEED_BRIDGE_POSITION = drive.trajectorySequenceBuilder(THREE_PIXEL_ON_BACKDROP.end())
//                .lineTo(Vector2dMapped(33.24, 12.25))
//                .lineTo(Vector2dMapped(-52.22, 12.98))
//                .build();
//
//        // Back 2 The Backboard
//        TrajectorySequence FIVE_INTAKE_PIXELS_STACK = drive.trajectorySequenceBuilder(FOUR_TO_LIGHTSPEED_BRIDGE_POSITION.end())
//                .lineTo(Vector2dMapped(-54.29, 12.98))
//                .build();
//
//        TrajectorySequence SIX_LIGHTSPEED_BRIDGE_BACK = drive.trajectorySequenceBuilder(FIVE_INTAKE_PIXELS_STACK.end())
//                .lineTo(Vector2dMapped(39.84, 12.11))
//                .build();
//
//        TrajectorySequence SEVEN_PIXEL_ON_BACKBOARD = drive.trajectorySequenceBuilder(SIX_LIGHTSPEED_BRIDGE_BACK.end())
//                .lineTo(Vector2dMapped(42.80, 31.76))
//                .lineTo(Vector2dMapped(53.85, 36.99))
//                .build();
//
//        TrajectorySequence EIGHT_PARK_END = drive.trajectorySequenceBuilder(SEVEN_PIXEL_ON_BACKBOARD.end())
//                .lineTo(Vector2dMapped(48.72, 26.24))
//                .lineTo(Vector2dMapped(51.54, 11.98))
//                .lineTo(Vector2dMapped(64.06, 12.65))
//                .build();

        schedule(
                new SequentialCommandGroup(
                        new FollowPath(pfollower, middleSpikeMark),

                        new FollowPath(pfollower, middleBackboardTraj),

                        new FollowPath(pfollower, middleStackTraj),

                        new FollowPath(pfollower, middleBackboardStackTraj),


                        new FollowPath(pfollower, middleStack2Traj),

                        new FollowPath(pfollower, middleBackboardStack2Traj),


                        new FollowPath(pfollower, middleStack3Traj),
                        new FollowPath(pfollower, middleEndTraj)
                )
        );
    };

//    @Override
//    public void run() {
//        super.run();
//        CommandScheduler.getInstance().run();
//        telemetry.update();
//    }


    private double HeadingMapped (double heading) {
        double mapper = rotationConstant;
        return heading + mapper;
    }

    private double HMapRadians (double headingDeg) {
        return HeadingMapped(Math.toRadians(headingDeg));
    }
    private double MathtoRadians (double toRad) {
        return HMapRadians(toRad);
    }

    // Wrapper class for bogus mogus
    public Vector2d Vector2dMapped(double x, double y) {
        com.acmerobotics.roadrunner.geometry.Vector2d conversionVector = new com.acmerobotics.roadrunner.geometry.Vector2d(x,y);
        conversionVector = conversionVector.rotated(rotationConstant);
        Vector2d returnVector = new Vector2d(conversionVector.getX(), conversionVector.getY());
        return returnVector;
    }
    public Pose2d Pose2dMapped(double x, double y, double heading) {
        com.acmerobotics.roadrunner.geometry.Vector2d conversionVector = new com.acmerobotics.roadrunner.geometry.Vector2d(x,y);
        conversionVector = conversionVector.rotated(rotationConstant);
        return new Pose2d(conversionVector.getX(), conversionVector.getY(), HeadingMapped(heading));
    }

    public Pose2d Pose2dMapped(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        double heading = pose.getHeading();
        com.acmerobotics.roadrunner.geometry.Vector2d conversionVector = new com.acmerobotics.roadrunner.geometry.Vector2d(x,y);
        conversionVector = conversionVector.rotated(rotationConstant);
        return new Pose2d(conversionVector.getX(), conversionVector.getY(), HeadingMapped(heading));
    }

}