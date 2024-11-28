package org.firstinspires.ftc.teamcode.tessy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.shared.common.RobotOpMode;
import org.firstinspires.ftc.teamcode.shared.tasks.GoToTask;
import org.firstinspires.ftc.teamcode.shared.tasks.Task;

import java.util.ArrayDeque;

@Autonomous(name = "Tessy Odometry Autonomous")
public class TessyAutonomousMecanum extends RobotOpMode {

    private static final double TICKS_PER_REV = 8192;
    private static final double WHEEL_DIAMETER_MM = 96;
    private static final double MM_PER_TICK = (Math.PI * WHEEL_DIAMETER_MM) / TICKS_PER_REV;

    private double xPosition = 0;
    private double yPosition = 0;
    private double lastLeftEncoder = 0;
    private double lastRightEncoder = 0;
    private double lastHorizontalEncoder = 0;
    private ArrayDeque<Task> tasks = new ArrayDeque<>();
    private TessyConfiguration config;
    private NormalisedMecanumDrive drive;

    @Override
    protected void onInit() {
        config = TessyConfiguration.newConfig(hardwareMap, telemetry);
        drive = new NormalisedMecanumDrive(
                this,
                config.leftBackMotor,
                config.rightBackMotor,
                config.leftFrontMotor,
                config.rightFrontMotor, true);


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
            tasks.removeFirst();

        }
        if (tasks.isEmpty()) {
            drive.setSpeedXYR(0, 0, 0 );
            drive.update();
        }
    }
}