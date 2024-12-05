package org.firstinspires.ftc.teamcode.shared.tasks;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.shared.common.RobotOpMode;
import org.firstinspires.ftc.teamcode.tessy.NormalisedMecanumDrive;

public class GoToTask extends BaseTask  implements Task {
    private final NormalisedMecanumDrive drive;
    private final SparkFunOTOS odometry;
    private final double target_x;
    private final double target_y;
    private final double target_error;
    private SparkFunOTOS.Pose2D current;
    private double distance;

    public GoToTask(RobotOpMode opMode, double time, NormalisedMecanumDrive drive, SparkFunOTOS odometry, double x, double y, double error) {
        super(opMode, time);

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
        opMode.telemetry.addLine("distance " + distance);
    }

    @Override
    public void run() {
        current = odometry.getPosition();


        double deltax = target_x - current.x;
        double deltay = target_y - current.y;
        distance = Math.sqrt(deltax*deltax+deltay*deltay);
        double angle = Math.atan2(deltay, deltax);
        double speed = 0.2;

        if (distance < 300) {
            speed = distance/300;
            if (speed < 0.1) {
                speed = 0.1;
            }
        }

        opMode.telemetry.addLine("Distance: " + distance);
        opMode.telemetry.addLine("Angle: " + angle);
        opMode.telemetry.addLine("Speed: " + speed);
        opMode.telemetry.update();
        if (isFinished()) {
            return;
        }

        drive.setSpeedXYR(deltax/distance, deltay/distance, 0);
//        drive.setSpeedXYR(deltax/distance, 0, 0);
        drive.update();
    }

    @Override
    public boolean isFinished() {

        return distance < target_error;
    }
}
