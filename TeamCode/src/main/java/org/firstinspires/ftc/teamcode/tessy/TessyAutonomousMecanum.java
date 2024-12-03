package org.firstinspires.ftc.teamcode.tessy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.shared.common.RobotOpMode;
import org.firstinspires.ftc.teamcode.shared.tasks.GoToTask;
import org.firstinspires.ftc.teamcode.shared.tasks.Task;
import java.util.ArrayDeque;

@Autonomous(name = "Tessy Odometry Autonomous")
@Disabled
public class TessyAutonomousMecanum extends RobotOpMode {

    private static final double TICKS_PER_REV = 8192;
    private static final double WHEEL_DIAMETER_MM = 96;
    private static final double MM_PER_TICK = (Math.PI * WHEEL_DIAMETER_MM) / TICKS_PER_REV;

    private ArrayDeque<Task> tasks = new ArrayDeque<>();
    private TessyConfiguration config;
    private NormalisedMecanumDrive drive;

    @Override
    protected void onInit() {
        // Initialize robot configuration and drive
        config = TessyConfiguration.newConfig(hardwareMap, telemetry);
        drive = new NormalisedMecanumDrive(
                this,
                config.leftBackMotor,
                config.rightBackMotor,
                config.leftFrontMotor,
                config.rightFrontMotor,
                true
        );

        // Add tasks for movement
        tasks.add(new GoToTask(drive, config.odometry, 500, 1000, 100));
        tasks.add(new GoToTask(drive, config.odometry, 1000, 500, 100));
    }

    @Override
    protected void activeLoop() throws InterruptedException {
        Task currentTask = tasks.peekFirst();
        if (currentTask == null) {
            return;
        }

        currentTask.run();
        if (currentTask.isFinished()) {
            tasks.removeFirst(); // Remove completed task
        }

        if (tasks.isEmpty()) {
            drive.setSpeedXYR(0, 0, 0);
            drive.update();
        }
    }
}
