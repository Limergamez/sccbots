package org.firstinspires.ftc.teamcode.shared.tasks;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.tessy.NormalisedMecanumDrive;

public class GoToTask implements Task {
    private final NormalisedMecanumDrive drive;
    private final SparkFunOTOS odometry;
    private final double targetX;
    private final double targetY;
    private final double error;
    private double posX;
    private double posY;
    private double distance;

    public GoToTask(NormalisedMecanumDrive drive, SparkFunOTOS odometry, double targetX, double targetY, double speed) {
        this.drive = drive;
        this.odometry = odometry;
        this.targetX = targetX;
        this.targetY = targetY;
        this.error = speed;
        this.posX = odometry.getPosition().x;
        this.posY = odometry.getPosition().y;
    }

    @Override
    public void init() {

    }

    @Override
    public void run() {
        double deltaX = targetX - posX;
        double deltaY = targetY - posY;
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        double angle = Math.atan2(deltaY, deltaX);
        drive.setSpeedPolarR(0.75, angle, 0);
    }

    @Override
    public boolean isFinished() {
        return distance <= error;
    }

}
