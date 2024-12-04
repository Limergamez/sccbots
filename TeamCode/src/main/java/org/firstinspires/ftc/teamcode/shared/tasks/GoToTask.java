package org.firstinspires.ftc.teamcode.shared.tasks;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.tessy.NormalisedMecanumDrive;

public class GoToTask implements Task {
    private final NormalisedMecanumDrive drive;
    private final SparkFunOTOS odometry;
    private final double targetX;
    private final double targetY;
    private final double speed;
    private double posX;
    private double posY;

    public GoToTask(NormalisedMecanumDrive drive, SparkFunOTOS odometry, double targetX, double targetY, double speed) {
        this.drive = drive;
        this.odometry = odometry;
        this.targetX = targetX;
        this.targetY = targetY;
        this.speed = speed;
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

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
