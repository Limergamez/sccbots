
package org.firstinspires.ftc.teamcode.tessy;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.shared.common.RobotOpMode;
import org.firstinspires.ftc.teamcode.shared.tasks.GoToTask;
import org.firstinspires.ftc.teamcode.shared.tasks.MessageTask;
import org.firstinspires.ftc.teamcode.shared.tasks.Task;
import org.firstinspires.ftc.teamcode.tessy.TessyConfiguration;

import java.util.ArrayDeque;

@Autonomous(name = "Bessy Autonomous")
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
                config.leftFrontMotor,
                config.rightFrontMotor,
                config.leftBackMotor,
                config.rightBackMotor, true);
        configureOtos();
        tasks.add(new MessageTask(this,1, "Starting"));
        tasks.add(new GoToTask(this, 3.0,
                drive, config.odometry, 100, 0, 10));
        tasks.add(new MessageTask(this,1, "done 1"));
        tasks.add(new GoToTask(this, 3.0,
                drive, config.odometry, 0, 100.0, 10));
        tasks.add(new MessageTask(this,1, "done 2"));
        tasks.add(new MessageTask(this,1, "Finished"));

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
            drive.setSpeedXYR(0, 0, 0);
            drive.update();
        }
    }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        config.odometry.setLinearUnit(DistanceUnit.MM);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        config.odometry.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(30, 0, 180);
        config.odometry.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        config.odometry.setLinearScalar(1.0);
        config.odometry.setAngularScalar(1.0);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        config.odometry.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        config.odometry.resetTracking();
        telemetry.addLine("Finished configuring OTOS...");
        telemetry.update();

   }
}

