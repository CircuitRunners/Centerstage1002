package org.firstinspires.ftc.teamcode.controllers.auto.pedrocommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.follower.Follower;


public class WaitForReachedTValue extends CommandBase {
    private final Follower m_follower;
    private final double m_tValue;
    private boolean m_isFinished = false;

    public WaitForReachedTValue(Follower follower, double tValue) {
        m_follower = follower;
        m_tValue = tValue;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (m_follower.getCurrentTValue() >= m_tValue) {
            m_isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return m_isFinished;
    }
}
