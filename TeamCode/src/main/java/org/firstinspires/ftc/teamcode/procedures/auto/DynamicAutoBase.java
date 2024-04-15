package org.firstinspires.ftc.teamcode.procedures.auto;

import static org.firstinspires.ftc.teamcode.controllers.Constants.AutoPoses.RED_AUDIENCE.r_a_startPos;
import static org.firstinspires.ftc.teamcode.controllers.Constants.AutoPoses.RED_STAGE.backPoint;
import static org.firstinspires.ftc.teamcode.controllers.Constants.AutoPoses.RED_STAGE.boardPositions;
import static org.firstinspires.ftc.teamcode.controllers.Constants.AutoPoses.RED_STAGE.parkPosition;
import static org.firstinspires.ftc.teamcode.controllers.Constants.AutoPoses.RED_STAGE.pixelPositions;
import static org.firstinspires.ftc.teamcode.controllers.Constants.AutoPoses.RED_STAGE.r_s_startPos;
import static org.firstinspires.ftc.teamcode.controllers.Constants.AutoPoses.RED_STAGE.stagePosition;
import static org.firstinspires.ftc.teamcode.controllers.common.utilities.Side.AUDIENCE;
import static org.firstinspires.ftc.teamcode.controllers.common.utilities.Side.BACKSTAGE;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.controllers.Constants;
import org.firstinspires.ftc.teamcode.controllers.auto.pedrocommands.FollowPath;
import org.firstinspires.ftc.teamcode.controllers.auto.pedrocommands.HoldPoint;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.controllers.commands.intake.IntakeCommandEx;
import org.firstinspires.ftc.teamcode.controllers.commands.presets.MoveToScoringCommandEx;
import org.firstinspires.ftc.teamcode.controllers.commands.presets.RetractOuttakeCommand;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.CrossBounds;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.PropLocation;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.Side;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.Team;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Intake;
import org.firstinspires.ftc.teamcode.procedures.AutoBase;

import java.util.Objects;

@Config
public class DynamicAutoBase extends AutoBase {
    public static int repeatCount = 2;

    @Override
    public SequentialCommandGroup build(Team team, Side side, PropLocation locations) {
        CrossBounds crossBound = CrossBounds.findCrossBound(team, side, locations);

        switch (team) {
            case RED: {
                if (side == AUDIENCE) {
                    robot.drive.setStartingPose(r_a_startPos);
                } else if (side == BACKSTAGE) {
                    robot.drive.setStartingPose(r_s_startPos);
                }
            }
            case BLUE: {
                if (side == AUDIENCE) {
//                    robot.drive.setStartingPose(r_s_startPos);
                } else if (side == BACKSTAGE) {
//                    robot.drive.setStartingPose(r_s_startPos);
                }
            }
        }

        switch (Objects.requireNonNull(
                crossBound
        )) {
            case RED_STAGE_MIDDLE: {
                return new SequentialCommandGroup(
                        new FollowPath(robot.drive, createPathChain(r_s_startPos, pixelPositions.getMiddle())),
                        new FollowPath(robot.drive, backwardsPathChain(pixelPositions.getMiddle(), backPoint.getMiddle())),
                        new FollowPath(robot.drive, createPathChain(backPoint.getMiddle(), boardPositions.getMiddle())),
                        new HoldPoint(robot.drive, boardPositions.getMiddle()),
                        new MoveToScoringCommandEx(robot.lift, robot.arm, robot.claw, MoveToScoringCommandEx.Presets.BOTTOM, robot.pivot),
                        new WaitCommand(700),
                        new InstantCommand(robot.claw::open),
                        new WaitCommand(700),
                        new RetractOuttakeCommand(robot.lift, robot.arm, robot.claw, robot.pivot),
                        new WaitCommand(500),
                        new FollowPath(robot.drive, createPathChain(boardPositions.getMiddle(), stagePosition)),
                        new FollowPath(robot.drive, createPathChain(stagePosition, parkPosition))
                );
            }
            case RED_AUDIENCE_MIDDLE: {
                return new SequentialCommandGroup(
                        new FollowPath(robot.drive, createPathChain(r_a_startPos, Constants.AutoPoses.RED_AUDIENCE.pixelPositions.getMiddle())),
                        new HoldPoint(robot.drive, Constants.AutoPoses.RED_AUDIENCE.pixelPositions.getMiddle()),
                        new WaitCommand(400),
                        new FollowPath(robot.drive, createPathChain(Constants.AutoPoses.RED_AUDIENCE.pixelPositions.getMiddle(), Constants.AutoPoses.RED_AUDIENCE.stackPositions)),
                        new InstantCommand(robot.frontArm::toPixel1),
                        new IntakeCommandEx(hardwareMap, robot.claw,robot.intake, Intake.IntakePowers.FAST),
                        new WaitCommand(1000), // stack
                        new FollowPath(robot.drive, createPathChain(
                                Constants.AutoPoses.RED_AUDIENCE.stackPositions,
                                Constants.AutoPoses.RED_AUDIENCE.beforeGoingThroughBridge)
                        ),
                        new FollowPath(robot.drive, createPathChain(
                                Constants.AutoPoses.RED_AUDIENCE.beforeGoingThroughBridge,
                                Constants.AutoPoses.RED_AUDIENCE.afterGoingThroughBridge)
                        ),
                        new FollowPath(robot.drive, createPathChain(
                                Constants.AutoPoses.RED_AUDIENCE.afterGoingThroughBridge,
                                Constants.AutoPoses.RED_AUDIENCE.toBoard.getMiddle())
                        ),
                        new HoldPoint(robot.drive, Constants.AutoPoses.RED_AUDIENCE.toBoard.getMiddle()),
                        new WaitCommand(1000), // stuff at the board
                        new FollowPath(robot.drive, createPathChain(
                                Constants.AutoPoses.RED_AUDIENCE.toBoard.getMiddle(),
                                Constants.AutoPoses.RED_AUDIENCE.toPark)
                        ),
                        new HoldPoint(robot.drive, Constants.AutoPoses.RED_AUDIENCE.toPark)
                );
            }
            case RED_AUDIENCE_RIGHT: {
                return new SequentialCommandGroup(
                        new FollowPath(robot.drive, createPathChain(r_a_startPos, Constants.AutoPoses.RED_AUDIENCE.pixelPositions.getRight())),
                        new HoldPoint(robot.drive, Constants.AutoPoses.RED_AUDIENCE.pixelPositions.getRight()),
                        new WaitCommand(400),
                        new FollowPath(robot.drive, createPathChain(Constants.AutoPoses.RED_AUDIENCE.pixelPositions.getMiddle(), Constants.AutoPoses.RED_AUDIENCE.stackPositions)),
                        new WaitCommand(1000), // stack
                        new FollowPath(robot.drive, createPathChain(
                                Constants.AutoPoses.RED_AUDIENCE.stackPositions,
                                Constants.AutoPoses.RED_AUDIENCE.beforeGoingThroughBridge)
                        ),
                        new FollowPath(robot.drive, createPathChain(
                                Constants.AutoPoses.RED_AUDIENCE.beforeGoingThroughBridge,
                                Constants.AutoPoses.RED_AUDIENCE.afterGoingThroughBridge)
                        ),
                        new FollowPath(robot.drive, createPathChain(
                                Constants.AutoPoses.RED_AUDIENCE.afterGoingThroughBridge,
                                Constants.AutoPoses.RED_AUDIENCE.toBoard.getRight())
                        ),
                        new HoldPoint(robot.drive, Constants.AutoPoses.RED_AUDIENCE.toBoard.getRight()),
                        new WaitCommand(1000), // stuff at the board
                        new FollowPath(robot.drive, createPathChain(
                                Constants.AutoPoses.RED_AUDIENCE.toBoard.getRight(),
                                Constants.AutoPoses.RED_AUDIENCE.toPark)
                        ),
                        new HoldPoint(robot.drive, Constants.AutoPoses.RED_AUDIENCE.toPark)
                );
            }
            case RED_AUDIENCE_LEFT: {
                return new SequentialCommandGroup(
                        new FollowPath(robot.drive, createPathChain(r_a_startPos, Constants.AutoPoses.RED_AUDIENCE.pixelPositions.getLeft())),
                        new HoldPoint(robot.drive, Constants.AutoPoses.RED_AUDIENCE.pixelPositions.getLeft()),
                        new WaitCommand(400),
                        new FollowPath(robot.drive, createPathChain(Constants.AutoPoses.RED_AUDIENCE.pixelPositions.getMiddle(), Constants.AutoPoses.RED_AUDIENCE.stackPositions)),
                        new WaitCommand(1000), // stack
                        new FollowPath(robot.drive, createPathChain(
                                Constants.AutoPoses.RED_AUDIENCE.stackPositions,
                                Constants.AutoPoses.RED_AUDIENCE.beforeGoingThroughBridge)
                        ),
                        new FollowPath(robot.drive, createPathChain(
                                Constants.AutoPoses.RED_AUDIENCE.beforeGoingThroughBridge,
                                Constants.AutoPoses.RED_AUDIENCE.afterGoingThroughBridge)
                        ),
                        new FollowPath(robot.drive, createPathChain(
                                Constants.AutoPoses.RED_AUDIENCE.afterGoingThroughBridge,
                                Constants.AutoPoses.RED_AUDIENCE.toBoard.getLeft())
                        ),
                        new HoldPoint(robot.drive, Constants.AutoPoses.RED_AUDIENCE.toBoard.getLeft()),
                        new WaitCommand(1000), // stuff at the board
                        new FollowPath(robot.drive, createPathChain(
                                Constants.AutoPoses.RED_AUDIENCE.toBoard.getLeft(),
                                Constants.AutoPoses.RED_AUDIENCE.toPark)
                        ),
                        new HoldPoint(robot.drive, Constants.AutoPoses.RED_AUDIENCE.toPark)
                );
            }
        }
        return new SequentialCommandGroup();
    }

    @Override
    public void runOnStart() {

    }

    public PathChain backwardsPathChain (Pose2d startPath, Pose2d endPath) {
        return createPathChain(endPath, startPath);
    }

    public PathChain createPathChain(Pose2d startPath, Pose2d endPath) {
        return robot.drive.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(startPath), new Point(endPath))))
                .setLinearHeadingInterpolation(startPath.getHeading(), endPath.getHeading())
                .build();
    }

}