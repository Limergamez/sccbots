package org.firstinspires.ftc.teamcode.bessy.otos;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.shared.common.RobotOpMode;
import org.firstinspires.ftc.teamcode.shared.tasks.BaseTask;
import org.firstinspires.ftc.teamcode.shared.tasks.Task;


public class GoToTask extends BaseTask implements Task {
    private final NormalisedMecanumDrive drive;
    private final SparkFunOTOS odometry;
    private final double target_x;
    private final double target_y;

    private final double target_h;

    private final double max_speed;
    private final double distance_error;
    private final double angle_error;
    private SparkFunOTOS.Pose2D current;
    private double distance;
    private double angle_distance;

    public GoToTask(RobotOpMode opMode, double time, NormalisedMecanumDrive drive, SparkFunOTOS odometry,
                    double x, double y, double h, double speed, double distance_error, double angle_error) {
        super(opMode, time);

        this.drive = drive;
        this.odometry = odometry;
        this.target_x = x;
        this.target_y = y;
        this.target_h = h;
        this.max_speed = speed;
        this.distance_error = distance_error;
        this.angle_error = angle_error;
    }

    @Override
    public void init() {
        current = odometry.getPosition();
        double deltax = target_x - current.x;
        double deltay = target_y - current.y;
        distance = Math.sqrt(deltax*deltax+deltay*deltay);
        angle_distance = target_h - current.h;
        opMode.telemetry.addLine("distance " + distance);
    }

    @Override
    public void run() {
        current = odometry.getPosition();


        double deltax = target_x - current.x;
        double deltay = target_y - current.y;
        distance = Math.sqrt(deltax*deltax+deltay*deltay);
        double angle = Math.atan2(deltay, deltax);
        angle_distance = unaliasAngle(target_h - current.h);
//        double speed = max_speed;
//        if (distance < 300) {
//            speed = distance/300;
//            if (speed < 0.1) {
//                speed = 0.1;
//            }
//        }
        double speed = Math.min(Math.abs(distance / 100)+0.1, max_speed) * Math.signum(distance);
        double max_delta_pos = Math.max(Math.abs(deltax),Math.abs(deltay));
        double speedx = (max_delta_pos > 0.0) ? deltax / max_delta_pos * speed : 0.0;
        double speedy = (max_delta_pos > 0.0) ? deltay / max_delta_pos * speed : 0.0;
        double speedr = Math.min(Math.abs(angle_distance / 30)+0.1, max_speed) * Math.signum(angle_distance);

        System.out.println(String.format("dX %4.2f sX: %4.2f  dY %4.2f sY: %4.2f  d %4.2f dh %4.1f h: %4.1f sH: %4.2f heading %4.1f", deltax, speedx, deltay, speedy, distance, angle_distance, current.h, speedr, angle*180.0/Math.PI));
        opMode.telemetry.addLine("Distance: " + distance);
        opMode.telemetry.addLine("Angle: " + angle);
        opMode.telemetry.addLine("Speed: " + max_speed);
        opMode.telemetry.update();
        if (isFinished()) {
            return;
        }


        drive.setSpeedXYR(speedx, -speedy, speedr);
//        drive.setSpeedXYR(deltax/distance, 0, 0);
        drive.update();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || (distance < distance_error && angle_distance < angle_error);
    }

    public static double unaliasAngle(double angle) {
        angle = angle % 360; // Reduce angle to the range -360 to 360
        if (angle > 180) {
            angle -= 360;
        } else if (angle <= -180) {
            angle += 360;
        }
        return angle;
    }
}
