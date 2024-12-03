package org.firstinspires.ftc.teamcode.shared.tasks;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.teamcode.tessy.NormalisedMecanumDrive;

public class GoToTask implements Task {
    private final NormalisedMecanumDrive drive;
    private final SparkFunOTOS odometry;
    private final double targetX;
    private final double targetY;
    private final double error;
    private double posX = 0;
    private double posY = 0;

    public GoToTask(NormalisedMecanumDrive drive, SparkFunOTOS odometry, double targetX, double targetY, double speed) {
        this.drive = drive;
        this.odometry = odometry;
        this.targetX = targetX;
        this.targetY = targetY;
        this.error = speed;
    }

    @Override
    public void init() {
    }

    @Override
    public void run() {
        if (odometry.getPosition() == null) {
            drive.setSpeedXYR(0, 0, 0);
            return;
        }
        posX = odometry.getPosition().x;
        posY = odometry.getPosition().y;

        double deltaX = targetX - posX;
        double deltaY = targetY - posY;
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        double angle = Math.atan2(deltaY, deltaX);

        drive.setSpeedPolarR(0.75, angle, 0);
    }

    @Override
    public boolean isFinished() {
        if (odometry.getPosition() == null) {
            return true;
        }
        double deltaX = targetX - odometry.getPosition().x;
        double deltaY = targetY - odometry.getPosition().y;
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY) <= error;
    }
}
