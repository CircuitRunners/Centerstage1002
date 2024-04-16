package org.firstinspires.ftc.teamcode.procedures;

import static org.firstinspires.ftc.teamcode.controllers.Constants.inCompetition;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controllers.VisionRobotCore;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.PropLocation;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.Side;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.Team;

@Config
public abstract class AutoBase extends CommandOpMode {
    public VisionRobotCore robot;

    public Team team;
    public Side side;
    public Follower drive;

    public PropLocation locationID;

    private SequentialCommandGroup mainCommandGroup;

    @Override
    public void initialize(){
        // Auto drive mechanics
        runOnStart();

        drive = new Follower(hardwareMap);

        robot = new VisionRobotCore(hardwareMap, team);

        robot.lift.initialInitHang();

        robot.detector.startStream();
        while (opModeInInit()) {
            locationID = robot.detector.update();
            telemetry.addData("Status", "In Init.");
            telemetry.addData("Prop", locationID.getLocation());
            telemetry.update();

//            sleep(30);
        }
        robot.detector.stopStream();
        inRuntime();
    }

    public void inRuntime () {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        mainCommandGroup = build(team, side, locationID);

        telemetry.addData("Loaded in", timer.milliseconds());
        telemetry.update();

        schedule(mainCommandGroup);
    }

    @Override
    public void run() {
        super.run();

        if (!inCompetition) {
            telemetry.addData("Save Money", drive.getPose());
        }

        drive.update();
        telemetry.update();
    }

    public abstract SequentialCommandGroup build (Team team, Side side, PropLocation locations);

    public abstract void runOnStart ();
}