package org.firstinspires.ftc.teamcode.bessy;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.shared.common.ArmControl;
import org.firstinspires.ftc.teamcode.shared.common.ClawControl;
import org.firstinspires.ftc.teamcode.shared.common.ClimbControl;
import org.firstinspires.ftc.teamcode.shared.common.LiftControl;
import org.firstinspires.ftc.teamcode.shared.common.RobotOpMode;
import org.firstinspires.ftc.teamcode.tessy.DualGamePadSteerDriveMecanum;
import org.firstinspires.ftc.teamcode.tessy.TessyConfiguration;

@TeleOp(name = "Bessy One Controller Mecanum")
public class BessyOneControllerDriveMecanum extends RobotOpMode {
    private TessyConfiguration config;
    private org.firstinspires.ftc.teamcode.tessy.DualGamePadSteerDriveMecanum drive;
    private LiftControl liftControl;
    private ClimbControl climbControl;
    private ClawControl clawControl;
    private ArmControl armControl;

    @Override
    protected void onInit() {
        config = TessyConfiguration.newConfig(hardwareMap, telemetry);

        drive = new DualGamePadSteerDriveMecanum(
                this,
                gamepad1,
                config.leftBackMotor,
                config.rightBackMotor,
                config.leftFrontMotor,
                config.rightFrontMotor
        );

        liftControl = new LiftControl(this, gamepad1, config.liftMotor);
        climbControl = new ClimbControl(this, gamepad1, config.climbMotor);
        clawControl = new ClawControl(this, gamepad1, config.clawServo);
        armControl = new ArmControl(this, gamepad1, config.armMotor);
    }

    @Override
    protected void activeLoop() throws InterruptedException {
        drive.update();
        liftControl.update();
        climbControl.update();
        clawControl.update();
        armControl.update();
        telemetry.update();
    }
}
