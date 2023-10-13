package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class TrajectoryCommand extends CommandBase {

    private SampleMecanumDrive drive;
    private Trajectory trajectory;

    public TrajectoryCommand(SampleMecanumDrive drive, Trajectory trajectory){
        this.drive = drive;
        this.trajectory = trajectory;
    }

    @Override
    public void initialize(){
        drive.followTrajectoryAsync(trajectory);
    }

    @Override
    public void execute(){
        drive.update();
    }

    /*
    public void end(boolean inturrupted) {
        do Things when done
    }
     */

    @Override
    public boolean isFinished(){
        return !drive.isBusy();
    }


}
