package org.firstinspires.ftc.teamcode.bessy.otos;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.bessy.BessyConfiguration;
import org.firstinspires.ftc.teamcode.shared.common.LiftControl;
import org.firstinspires.ftc.teamcode.shared.common.RobotOpMode;
import org.firstinspires.ftc.teamcode.shared.tasks.Task;
import org.firstinspires.ftc.teamcode.tessy.TessyConfiguration;

import java.util.ArrayDeque;

@Autonomous(name = "Task Based Mecanum Autonomous", group  = "Mecanum")
public class AutonomousMechanum extends RobotOpMode {
    private TessyConfiguration config;
    private NormalisedMecanumDrive drive;
    private DcMotor liftMotor;
    private ArrayDeque<Task> tasks = new ArrayDeque<>();

    private double heading;

    @Override
    protected void onInit() {
        config = BessyConfiguration.newConfig(hardwareMap, telemetry);

        drive = new NormalisedMecanumDrive(this,
                config.leftFrontMotor, config.rightFrontMotor,
                config.leftBackMotor, config.rightBackMotor, true);

        tasks.add(new GoToTask(this, 100, drive, config.odometry, 0, 779.2, 0, 0.5, 20, 5));
        tasks.add(new MotorTask(this, 1.23, liftMotor, -1, true));
        //arm motor task
        tasks.add(new MotorTask(this, 0.5, liftMotor, 1, true));



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