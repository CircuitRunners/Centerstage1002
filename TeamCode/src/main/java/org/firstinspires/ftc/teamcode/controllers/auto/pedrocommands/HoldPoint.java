package org.firstinspires.ftc.teamcode.controllers.auto.pedrocommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.BezierPoint;

public class HoldPoint extends CommandBase {
    private final Follower m_follower;
    private final BezierPoint m_point;
    private final double m_heading;

    public HoldPoint(Follower follower, BezierPoint point, double heading) {
        m_follower = follower;
        m_point = point;
        m_heading = heading;
    }

    @Override
    public void initialize() {
        m_follower.holdPoint(m_point, m_heading);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
