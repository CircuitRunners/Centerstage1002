package org.firstinspires.ftc.teamcode.auto.old;
import static org.firstinspires.ftc.teamcode.utilities.CrossBindings.DEFAULT_PROP_LOCATION;
import static org.firstinspires.ftc.teamcode.utilities.CrossBindings.rotationConstant;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeStackCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.commands.presets.MoveToScoringCommand;
import org.firstinspires.ftc.teamcode.commands.presets.RetractOuttakeCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.ExtendoArm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utilities.PropLocation;
import org.firstinspires.ftc.teamcode.utilities.Team;
import org.firstinspires.ftc.teamcode.vision.TeamPropDetector;

@Autonomous (name = "Bad Blue Audience")
public class SussyAuto extends CommandOpMode{

    private TrajectorySequence THREE_PIXEL_ON_BACKDROP;
    private TrajectorySequence PURPLE_GLOBAL;

    // Default Values
    private PropLocation locationID = DEFAULT_PROP_LOCATION; // set to center by default

    private Pose2d startPose = Pose2dMapped(-39.75, 63, Math.toRadians(270.00));

    // Subsystems initialization
    private Lift lift;
    private Arm arm;
    private Claw claw;
    private ExtendoArm extendo;
    private Intake intake;
    private TeamPropDetector detector;

    // Roadrunner Drive class
    private SampleMecanumDrive drive;


    @Override
    public void initialize (){
        schedule(new BulkCacheCommand(hardwareMap));

        detector = new TeamPropDetector(hardwareMap, true, Team.BLUE);
        intake = new Intake(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        lift = new Lift(hardwareMap);
        claw = new Claw(hardwareMap);
        arm = new Arm(hardwareMap);
        extendo = new ExtendoArm(hardwareMap);

        lift.initialInitHang();
        drive.setPoseEstimate(startPose);
        detector.startStream();

        while(opModeInInit()){
            // The location id returns either LEFT, MIDDLE, or RIGHT.
            // You will have to run location.getLocation() to retrieve the string of LEFT, etc.
            locationID = detector.update();

            telemetry.addLine("Ready for start!");
            telemetry.addData("Prop", locationID.getLocation());

            telemetry.update();
        }

        // Cleanup for other code
        detector.stopStream();

        switch(locationID){
            case LEFT: {
                PURPLE_GLOBAL = drive.trajectorySequenceBuilder(startPose)

                        .build();
            }
            case MIDDLE: {
                PURPLE_GLOBAL = drive.trajectorySequenceBuilder(startPose)
                        .lineToLinearHeading(Pose2dMapped(-39.75, 33.91, Math.toRadians(270)))
                        .lineTo(Vector2dMapped(-39.75, 36.66))
                        .build();
            }
            case RIGHT: {
                PURPLE_GLOBAL = drive.trajectorySequenceBuilder(startPose)
                        .build();
            }

    }

}
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
        Vector2d conversionVector = new Vector2d(x,y);
        conversionVector = conversionVector.rotated(rotationConstant);
        Vector2d returnVector = new Vector2d(conversionVector.getX(), conversionVector.getY());
        return returnVector;
    }
    public Pose2d Pose2dMapped(double x, double y, double heading) {
        Vector2d conversionVector = new Vector2d(x,y);
        conversionVector = conversionVector.rotated(rotationConstant);
        return new Pose2d(conversionVector.getX(), conversionVector.getY(), HeadingMapped(heading));
    }

    public Pose2d Pose2dMapped(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        double heading = pose.getHeading();
        Vector2d conversionVector = new Vector2d(x,y);
        conversionVector = conversionVector.rotated(rotationConstant);
        return new Pose2d(conversionVector.getX(), conversionVector.getY(), HeadingMapped(heading));
    }

}
