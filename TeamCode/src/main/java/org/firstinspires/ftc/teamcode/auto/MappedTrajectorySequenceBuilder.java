package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.utilities.CrossBindings.rotationConstant;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

public class MappedTrajectorySequenceBuilder {
    private final TrajectorySequenceBuilder builder;

    public MappedTrajectorySequenceBuilder(SampleMecanumDrive drive, Pose2d startPose) {
        // Apply mapping to the start pose
        Pose2d mappedStartPose = mapPose(startPose);
        this.builder = drive.trajectorySequenceBuilder(mappedStartPose);
    }

    public MappedTrajectorySequenceBuilder lineTo(Vector2d endPosition) {
        Vector2d mappedEndPosition = mapVector(endPosition);
        builder.lineTo(mappedEndPosition);
        return this;
    }

    public MappedTrajectorySequenceBuilder lineToLinearHeading(Pose2d endPose) {
        Pose2d mappedEndPose = mapPose(endPose);
        builder.lineToLinearHeading(mappedEndPose);
        return this;
    }

    public MappedTrajectorySequenceBuilder lineToSplineHeading(Pose2d endPose) {
        Pose2d mappedEndPose = mapPose(endPose);
        builder.lineToSplineHeading(mappedEndPose);
        return this;
    }

    // End Tangent just means the angle it ends at
    public MappedTrajectorySequenceBuilder splineTo(Vector2d endPosition, double endTangent) {
        Vector2d endPosMapped = mapVector(endPosition);
        double mappedEndPose = endTangent + rotationConstant;
        builder.splineTo(endPosMapped, mappedEndPose);
        return this;
    }

    // Implement other methods as needed...

    public TrajectorySequence build() {
        return builder.build();
    }

    private double HeadingMapped (double heading) {
        double mapper = rotationConstant;
        return heading + mapper;
    }

    public Pose2d mapPose(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        double heading = pose.getHeading();
        Vector2d conversionVector = new Vector2d(x,y);
        conversionVector = conversionVector.rotated(rotationConstant);
        return new Pose2d(conversionVector.getX(), conversionVector.getY(), HeadingMapped(heading));
    }

    private Vector2d mapVector(Vector2d vector) {
        // Assuming a simple rotation mapping for demonstration
        // Adjust this logic based on your actual mapping needs
        double newX = vector.getX() * Math.cos(rotationConstant) - vector.getY() * Math.sin(rotationConstant);
        double newY = vector.getX() * Math.sin(rotationConstant) + vector.getY() * Math.cos(rotationConstant);
        return new Vector2d(newX, newY);
    }

    public Vector2d Vector2dMapped(Vector2d vec) {
        Vector2d conversionVector = vec.rotated(rotationConstant);
        Vector2d returnVector = new Vector2d(conversionVector.getX(), conversionVector.getY());
        return returnVector;
    }

    public Vector2d Vector2dMapped(double x, double y) {
        Vector2d conversionVector = new Vector2d(x,y);
        return Vector2dMapped(conversionVector);
    }
}