package org.firstinspires.ftc.teamcode.shared.tasks;

import org.firstinspires.ftc.teamcode.shared.common.RobotOpMode;

public class WaitTask extends BaseTask implements Task {


    public WaitTask(RobotOpMode opMode, double time) {
        super(opMode, time);

    }

    void update() {

    }

    @Override
    public void run() {
        if (isFinished()) {
            return;
        }

    }

}
