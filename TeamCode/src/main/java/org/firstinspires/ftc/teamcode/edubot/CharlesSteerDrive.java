package org.firstinspires.ftc.teamcode.edubot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opencv.ColourCountVision;
import org.firstinspires.ftc.teamcode.shared.common.GamePadSteerDrive;
import org.firstinspires.ftc.teamcode.shared.common.RobotOpMode;
import org.opencv.core.Scalar;


@TeleOp(name = "Charles SteerDrive v9")
public class CharlesSteerDrive extends RobotOpMode {
    private CharlesConfiguration config;
    private GamePadSteerDrive drive;
    CharlesArmController armLift;
    ColourCountVision vision;

    @Override
    protected void onInit() {
        config = CharlesConfiguration.newConfig(hardwareMap, telemetry);
        drive = new GamePadSteerDrive(this, gamepad1, config.leftMotor, config.rightMotor);
        armLift = new CharlesArmController(this, gamepad1, config, 0.3f, true);
        vision = new ColourCountVision(hardwareMap, telemetry, 640, 480, new Scalar(127.0, 0.0, 0.0, 0.0), new Scalar(255.0, 127.0, 127.0, 255.0));
    }

    @Override
    protected void activeLoop() throws InterruptedException {
        drive.update();
        armLift.update();

        telemetry.addData("Object in zone:", vision.colourZone());
        telemetry.update();
    }
}