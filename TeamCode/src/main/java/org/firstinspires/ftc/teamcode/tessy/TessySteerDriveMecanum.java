package org.firstinspires.ftc.teamcode.tessy;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.shared.common.RobotOpMode;

@TeleOp(name = "Tessy Mecanum Drive")
public class TessySteerDriveMecanum extends RobotOpMode {
    private TessyConfiguration config;
    private DualGamePadSteerDriveMecanum drive;

    @Override
    protected void onInit() {
        config = TessyConfiguration.newConfig(hardwareMap, telemetry);
        drive = new DualGamePadSteerDriveMecanum(this, gamepad1, config.leftBackMotor, config.rightBackMotor, config.leftFrontMotor, config.rightFrontMotor);
    }

    @Override
    protected void activeLoop() throws InterruptedException {
        drive.update();
        telemetry.update();
    }
}
