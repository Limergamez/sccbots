package org.firstinspires.ftc.teamcode.bessy.otos;

import org.firstinspires.ftc.teamcode.shared.common.RobotOpMode;
import org.firstinspires.ftc.teamcode.shared.tasks.BaseTask;
import org.firstinspires.ftc.teamcode.shared.tasks.Task;


public class MecanumDriveTask extends BaseTask implements Task {
    private final NormalisedMecanumDrive drive;
    private final double x;
    private final double y;
    private final double r;

    public MecanumDriveTask(RobotOpMode opMode, double time,
                            NormalisedMecanumDrive drive,
                            double x, double y, double r) {
        super(opMode, time);

        this.drive = drive;
        this.x = x;
        this.y = y;
        this.r = r;
    }

    @Override
    public void init() {

    }

    @Override
    public void run() {

        if (isFinished()) {
            return;
        }

        opMode.telemetry.addLine("X: " + x);
        opMode.telemetry.addLine("Y: " + y);
        opMode.telemetry.addLine("R: " + r);
        opMode.telemetry.update();

        drive.setSpeedXYR(x, y, r);
        drive.update();
    }

}
