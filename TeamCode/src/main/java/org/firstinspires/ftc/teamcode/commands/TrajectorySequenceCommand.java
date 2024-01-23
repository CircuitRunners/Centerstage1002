package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class TrajectorySequenceCommand extends CommandBase {

    private MecanumDrive drive;
    private Action trajectory;

    public TrajectorySequenceCommand(MecanumDrive drive, Action trajectory){
        this.drive = drive;
        this.trajectory = trajectory;
    }

    @Override
    public void initialize(){
        Actions.runBlocking(trajectory);
    }

    @Override
    public void execute(){
        drive.update();
    }

    @Override
    public boolean isFinished(){
        return !drive.isBusy();
    }


}