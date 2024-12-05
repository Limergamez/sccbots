package org.firstinspires.ftc.teamcode.shared.tasks;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.tessy.NormalisedMecanumDrive;

public class GoToTask implements Task {
    private final NormalisedMecanumDrive drive;
    private final SparkFunOTOS odometry;
    private final double target_x;
    private final double target_y;
    private final double target_error;
    private SparkFunOTOS.Pose2D current;
    private double distance;

    public GoToTask(NormalisedMecanumDrive drive, SparkFunOTOS odometry, double x, double y, double error) {
        this.drive = drive;
        this.odometry = odometry;
        this.target_x = x;
        this.target_y = y;
        this.target_error = error;
    }

    @Override
    public void init() {
        current = odometry.getPosition();
        double deltax = target_x - current.x;
        double deltay = target_y - current.y;
        distance = Math.sqrt(deltax*deltax+deltay*deltay);
    }

    @Override
    public void run() {
        current = odometry.getPosition();
        double deltax = target_x - current.x;
        double deltay = target_y - current.y;
        distance = Math.sqrt(deltax*deltax+deltay*deltay);
        double angle = Math.atan2(deltay, deltax);
        double speed = 1;

        if (distance < 300) {
            speed = distance/300;
            if (speed < 0.2) {
                speed = 0.2;
            }
        }

        drive.setSpeedPolarR(speed, angle, 0);
    }

    @Override
    public boolean isFinished() {
        return distance < target_error;
    }
}
