package org.firstinspires.ftc.teamcode.controllers.common.utilities;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class TriPose {
    private final Pose2d left;
    private final Pose2d middle;
    private final Pose2d right;

    public TriPose(Pose2d left, Pose2d middle, Pose2d right) {
        this.left = left;
        this.middle = middle;
        this.right = right;
    }

    public Pose2d getLeft() {
        return left;
    }

    public Pose2d getMiddle() {
        return middle;
    }

    public Pose2d getRight() {
        return right;
    }

    @Override
    public String toString() {
        return "TriPositions{" +
                "left=" + left +
                ", middle=" + middle +
                ", right=" + right +
                '}';
    }
}