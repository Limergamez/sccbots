package org.firstinspires.ftc.teamcode.edubot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opencv.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.shared.common.GamePadSteerDrive;
import org.firstinspires.ftc.teamcode.shared.common.RobotOpMode;
import org.firstinspires.ftc.teamcode.shared.tasks.MessageTask;
import org.firstinspires.ftc.teamcode.shared.tasks.Task;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.TreeMap;

@Autonomous(name = "No OpenCV")
public class CharlesOpenCVAutonomous extends RobotOpMode {

    private CharlesConfiguration config;
    private GamePadSteerDrive drive;
    CharlesArmController armLift;
    private ArrayDeque<Task> tasks = new ArrayDeque<>();

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.05;

    int ID_TAG_OF_INTEREST = 111;
    TreeMap<Integer, Integer> TAGS_OF_INTEREST = new TreeMap<>();

    AprilTagDetection tagOfInterest = null;

    @Override
    protected void onInit() {
        config = CharlesConfiguration.newConfig(hardwareMap, telemetry);
        drive = new GamePadSteerDrive(this, gamepad1, config.leftMotor, config.rightMotor);
        armLift = new CharlesArmController(this, gamepad1, config, 0.3f, true);

        TAGS_OF_INTEREST.put(111, 1);
        TAGS_OF_INTEREST.put(222, 2);
        TAGS_OF_INTEREST.put(333, 3);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        int count=0;
        while (tagOfInterest == null && !isStarted() && !isStopRequested()) {

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                for (AprilTagDetection tag : currentDetections) {
                    if (TAGS_OF_INTEREST.containsKey(tag.id)) {
                        tagOfInterest = tag;
                        telemetry.addLine(String.format("Tag id: %d", tag.id));
                        break;

                    } else {
                        telemetry.addLine(String.format("Unrecognised Tag id: %d", tag.id));
                    }
                }

            } else {
                telemetry.addLine(String.format("No detections: %d", count));
            }
            count++;
            telemetry.update();


            sleep(20);
        }

        if (tagOfInterest == null) {
            tasks.add(new MessageTask(this, 1, "No Tag Found"));
        } else {
            tasks.add(new MessageTask(this, 2, String.format("Found tag %d", tagOfInterest.id)));
            ;
        }

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
            config.leftMotor.setPower(0);
            config.rightMotor.setPower(0);
            config.armMotor.setPower(0);
        }
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.0f mm", detection.pose.x * 1000));
        telemetry.addLine(String.format("Translation Y: %.0f mm", detection.pose.y * 1000));
        telemetry.addLine(String.format("Translation Z: %.0f mm", detection.pose.z * 1000));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
