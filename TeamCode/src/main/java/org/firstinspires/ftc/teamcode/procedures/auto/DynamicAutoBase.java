package org.firstinspires.ftc.teamcode.procedures.auto;

import static org.firstinspires.ftc.teamcode.controllers.Constants.AutoPoses.RED_AUDIENCE.r_a_startPos;
import static org.firstinspires.ftc.teamcode.controllers.Constants.AutoPoses.RED_AUDIENCE.stackPositions;
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
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.controllers.Constants;
import org.firstinspires.ftc.teamcode.controllers.RobotCore;
import org.firstinspires.ftc.teamcode.controllers.auto.pedrocommands.FollowPath;
import org.firstinspires.ftc.teamcode.controllers.auto.pedrocommands.HoldPoint;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.controllers.commands.intake.IntakeCommandEx;
import org.firstinspires.ftc.teamcode.controllers.commands.presets.MoveToScoringCommandEx;
import org.firstinspires.ftc.teamcode.controllers.commands.presets.RetractOuttakeCommand;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.PropLocation;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.Side;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.Team;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.TriPose;
import org.firstinspires.ftc.teamcode.controllers.subsytems.ExtendoArm;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Intake;
import org.firstinspires.ftc.teamcode.procedures.AutoBase;

@Config
public class DynamicAutoBase extends AutoBase {
    public static int repeatCount = 2;

    @Override
    public SequentialCommandGroup build(Team team, Side side, PropLocation locations) {
        switch (team) {
            case RED: {
                if (side == AUDIENCE) {
                    // RED AUDIENCE
                    drive.setStartingPose(r_a_startPos);
                    return new SequentialCommandGroup(
                            new FollowPath(drive, createPathChain(r_a_startPos, getPosition(Constants.AutoPoses.RED_AUDIENCE.pixelPositions, locations))),
                            new WaitCommand(400),
                            new HoldPoint(drive, getPosition(Constants.AutoPoses.RED_AUDIENCE.pixelPositions, locations)),
                            new WaitCommand(400),
                            new InstantCommand(robot.frontArm::toPixel1),
                            new FollowPath(drive, createPathChain(getPosition(Constants.AutoPoses.RED_AUDIENCE.pixelPositions, locations), Constants.AutoPoses.RED_AUDIENCE.stackPositions)),
                            new HoldPoint(drive, stackPositions),
                            new WaitCommand(400),
                            new IntakeCommandEx(hardwareMap, robot.claw, robot.intake, robot.arm, Intake.IntakePowers.FAST),
                            new WaitCommand(1000), // stack

                            new FollowPath(drive, createPathChain(
                                    Constants.AutoPoses.RED_AUDIENCE.stackPositions,
                                    Constants.AutoPoses.RED_AUDIENCE.beforeGoingThroughBridge)
                            ),
                            new FollowPath(drive, createPathChain(
                                    Constants.AutoPoses.RED_AUDIENCE.beforeGoingThroughBridge,
                                    Constants.AutoPoses.RED_AUDIENCE.afterGoingThroughBridge)
                            ),
                            new FollowPath(drive, createPathChain(
                                    Constants.AutoPoses.RED_AUDIENCE.afterGoingThroughBridge,
                                    getPosition(Constants.AutoPoses.RED_AUDIENCE.preBoard, locations))
                            ),
                            new ParallelCommandGroup(
                                    new HoldPoint(drive, getPosition(Constants.AutoPoses.RED_AUDIENCE.preBoard, locations)),
                                    new MoveToScoringCommandEx(robot.lift, robot.arm, robot.claw, MoveToScoringCommandEx.Presets.SHORT, robot.pivot)
                            ),
                            new InstantCommand(() -> {
                                if (locations != PropLocation.RIGHT) {
                                    robot.pivot.right();
                                } else {
                                    robot.pivot.left();
                                }
                            }),
                            new WaitCommand(400),
                            new FollowPath(drive, createPathChain(
                                    getPosition(Constants.AutoPoses.RED_AUDIENCE.preBoard, locations),
                                    getPosition(Constants.AutoPoses.RED_AUDIENCE.toBoard, locations))
                            ),
                            new WaitCommand(1000),
                            new InstantCommand(() -> {
                                if (locations != PropLocation.RIGHT) {
                                    robot.pivot.left();
                                } else {
                                    robot.pivot.right();
                                }
                            }),
                            new WaitCommand(1000),
                            new InstantCommand(robot.claw::open),
                            new WaitCommand(1000),
                            new RetractOuttakeCommand(robot.lift, robot.arm, robot.claw, robot.pivot),
                            new HoldPoint(drive, Constants.AutoPoses.RED_AUDIENCE.toPark)
                    );
                } else if (side == BACKSTAGE) {
                    // RED BACKSTAGE
                    drive.setStartingPose(r_s_startPos);
                    return new SequentialCommandGroup(
                            new FollowPath(drive, createPathChain(r_s_startPos, getPosition(pixelPositions, locations))), // to pixel point
                            new WaitCommand(400),
                            new FollowPath(drive, backwardsPathChain(getPosition(pixelPositions, locations), getPosition(backPoint, locations))), // move back to avoid moving pixel
                            new FollowPath(drive, createPathChain(getPosition(backPoint, locations), getPosition(boardPositions, locations))), // go to the board
                            new HoldPoint(drive, getPosition(boardPositions, locations)), // chill at the board for accuracy on heading
                            presetToLiftPosition(robot, MoveToScoringCommandEx.Presets.BOTTOM),
                            new FollowPath(drive, createPathChain(getPosition(boardPositions, locations), stagePosition)), // go to the stage (from the board)
                            new FollowPath(drive, createPathChain(stagePosition, parkPosition)) // parque!
                    );
                }
            }
            case BLUE: {
                if (side == AUDIENCE) {
                    // BLUE AUDIENCE
                    drive.setStartingPose(mirrorPose(r_a_startPos));
                    return new SequentialCommandGroup(
                            new FollowPath(drive, mirroredPathChain(r_a_startPos, getPosition(Constants.AutoPoses.RED_AUDIENCE.pixelPositions, locations))),
                            new WaitCommand(400),
                            new HoldPoint(drive, getPositionMirrored(Constants.AutoPoses.RED_AUDIENCE.pixelPositions, locations)),
                            new WaitCommand(400),
                            new FollowPath(drive, mirroredPathChain(getPosition(Constants.AutoPoses.RED_AUDIENCE.pixelPositions, locations), Constants.AutoPoses.RED_AUDIENCE.stackPositions)),
                            new WaitCommand(1000), // stack
                            new FollowPath(drive, mirroredPathChain(
                                    Constants.AutoPoses.RED_AUDIENCE.stackPositions,
                                    Constants.AutoPoses.RED_AUDIENCE.beforeGoingThroughBridge)
                            ),
                            new FollowPath(drive, mirroredPathChain(
                                    Constants.AutoPoses.RED_AUDIENCE.beforeGoingThroughBridge,
                                    Constants.AutoPoses.RED_AUDIENCE.afterGoingThroughBridge)
                            ),
                            new FollowPath(drive, mirroredPathChain(
                                    Constants.AutoPoses.RED_AUDIENCE.afterGoingThroughBridge,
                                    getPosition(Constants.AutoPoses.RED_AUDIENCE.toBoard, locations))
                            ),
                            new HoldPoint(drive, getPositionMirrored(Constants.AutoPoses.RED_AUDIENCE.toBoard, locations)),
                            presetToLiftPosition(robot, MoveToScoringCommandEx.Presets.BOTTOM),
                            new WaitCommand(1000), // stuff at the board
                            new FollowPath(drive, mirroredPathChain(
                                    getPosition(Constants.AutoPoses.RED_AUDIENCE.toBoard, locations),
                                    Constants.AutoPoses.RED_AUDIENCE.toPark)
                            ),
                            new HoldPoint(drive, mirrorPose(Constants.AutoPoses.RED_AUDIENCE.toPark))
                    );
                } else if (side == BACKSTAGE) {
//                    drive.setStartingPose(r_s_startPos);
                }
            }
        }
        return new SequentialCommandGroup();
    }

    private SequentialCommandGroup presetToLiftPosition (RobotCore robot, MoveToScoringCommandEx.Presets preset) {
        return new SequentialCommandGroup(
                new MoveToScoringCommandEx(robot.lift, robot.arm, robot.claw, preset , robot.pivot), // lift go up
                new WaitCommand(350),
                new InstantCommand(robot.claw::open), // open the claw to drop
                new WaitCommand(700),
                new RetractOuttakeCommand(robot.lift, robot.arm, robot.claw, robot.pivot), // bring the outtake down
                new WaitCommand(500)
        );
    }

    private Pose2d getPosition(TriPose triPose, PropLocation side) {
        switch (side) {
            case LEFT:
                return triPose.getLeft();
            case RIGHT:
                return triPose.getRight();
            case MIDDLE:
            default:
                return triPose.getMiddle();
        }
    }

    private Pose2d getPositionMirrored (TriPose triPose, PropLocation side) {
        // Think about it, a right pixel on red == left pixel on blue
        switch (side) {
            case LEFT:
                return mirrorPose(triPose.getRight());
            case RIGHT:
                return mirrorPose(triPose.getLeft());
            case MIDDLE:
            default:
                return mirrorPose(triPose.getMiddle());
        }
    }

    private PropLocation reversePixeler (PropLocation side) {
        if (side == PropLocation.LEFT) {
            return PropLocation.RIGHT;
        } else if (side == PropLocation.RIGHT) {
            return PropLocation.LEFT;
        } else {
            return PropLocation.LEFT;
        }
    }

    private Pose2d mirrorPose(Pose2d originalPose) {
        // Mirror the x-coordinate and invert the heading
        double mirroredX = originalPose.getX();
        double mirroredY = -originalPose.getY();
        double mirroredHeading = -originalPose.getHeading(); // Convert heading to radians if necessary

        return new Pose2d(mirroredX, mirroredY, mirroredHeading);
    }

    /**
     * Creates a mirrored path chain from the red side to the blue side.
     * @param startPath Start pose on the red side.
     * @param endPath End pose on the red side.
     * @return Mirrored path chain for the blue side.
     */
    public PathChain mirroredPathChain(Pose2d startPath, Pose2d endPath) {
        // Mirror both start and end poses
        Pose2d mirroredStart = mirrorPose(startPath);
        Pose2d mirroredEnd = mirrorPose(endPath);

        // Create path chain with mirrored poses
        return createPathChain(mirroredStart, mirroredEnd);
    }

    public PathChain backwardsMirroredPathChain(Pose2d startPath, Pose2d endPath) {
        // Mirror both start and end poses
        Pose2d mirroredStart = mirrorPose(startPath);
        Pose2d mirroredEnd = mirrorPose(endPath);

        // Create path chain with mirrored poses
        return createPathChain(mirroredEnd, mirroredStart);
    }

    @Override
    public void runOnStart() {

    }

    public PathChain backwardsPathChain (Pose2d startPath, Pose2d endPath) {
        return createPathChain(endPath, startPath);
    }

    public PathChain createPathChain(Pose2d startPath, Pose2d endPath) {
        return drive.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(startPath), new Point(endPath))))
                .setLinearHeadingInterpolation(startPath.getHeading(), endPath.getHeading())
                .build();
    }

}