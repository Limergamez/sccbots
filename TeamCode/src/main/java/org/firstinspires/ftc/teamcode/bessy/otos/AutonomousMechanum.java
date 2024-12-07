package org.firstinspires.ftc.teamcode.bessy.otos;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.bessy.BessyConfiguration;
import org.firstinspires.ftc.teamcode.shared.common.RobotOpMode;
import org.firstinspires.ftc.teamcode.shared.tasks.Task;
import org.firstinspires.ftc.teamcode.tessy.TessyConfiguration;

import java.util.ArrayDeque;

@Autonomous(name = "Task Based Mecanum Autonomous", group  = "Mecanum")
public class AutonomousMechanum extends RobotOpMode {
    private TessyConfiguration config;
    private NormalisedMecanumDrive drive;
    private ArrayDeque<Task> tasks = new ArrayDeque<>();

    private double heading;

    @Override
    protected void onInit() {
        config = BessyConfiguration.newConfig(hardwareMap, telemetry);

        drive = new NormalisedMecanumDrive(this,
                config.leftFrontMotor, config.rightFrontMotor,
                config.leftBackMotor, config.rightBackMotor, true);

//        tasks.add(new MecanumDriveTask(this, 1, drive, 1, 0, 0));
//        tasks.add(new GoToTask(this, 4, drive, config.otos, 0, 600, 90, 0.5, 20, 5));
//        tasks.add(new GoToTask(this, 6, drive, config.otos, 600, 600, 181,0.5, 20, 5));
//        tasks.add(new GoToTask(this, 4, drive, config.otos, 0, 600, 269, 0.5, 20, 5));
//        tasks.add(new GoToTask(this, 4, drive, config.otos, 0, 0, 0, 0.5, 20, 5));
        tasks.add(new GoToTask(this, 4, drive, config.odometry, 0, 500, 0, 0.5, 20, 5));
        tasks.add(new GoToTask(this, 6, drive, config.odometry, 500, 500, 0,0.5, 20, 5));
        tasks.add(new GoToTask(this, 4, drive, config.odometry, 500, 0, 0, 0.5, 20, 5));
        tasks.add(new GoToTask(this, 4, drive, config.odometry, 0, 0, 0, 0.5, 20, 5));
//        tasks.add(new MecanumDriveTask(this, 1, drive, 0, 1, 0));
//        tasks.add(new MecanumDriveTask(this, 3, drive, 0, 0.1, 1));

        config.odometry.setOffset(new SparkFunOTOS.Pose2D(0,0,0));

    }

    @Override
    protected void activeLoop() throws InterruptedException {
        telemetry.addData("x", config.odometry.getPosition().x);
        telemetry.addData("y", config.odometry.getPosition().y);
        telemetry.addData("h", config.odometry.getPosition().h);
        Task currentTask = tasks.peekFirst();
        if (currentTask == null) {
            this.setOperationsCompleted();
            return;
        }
        currentTask.run();
        if (currentTask.isFinished()){
            tasks.removeFirst();

        }
        if (tasks.isEmpty()) {
            drive.setSpeedXYR(0, 0, 0);
            drive.update();
        }
    }
}