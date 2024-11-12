package org.firstinspires.ftc.teamcode.jeremy;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.shared.common.ArmControl;
import org.firstinspires.ftc.teamcode.shared.common.ClawControl;
import org.firstinspires.ftc.teamcode.shared.common.ClimbControl;
import org.firstinspires.ftc.teamcode.shared.common.DualGamePadSteerDrive;
import org.firstinspires.ftc.teamcode.shared.common.LiftControl;
import org.firstinspires.ftc.teamcode.shared.common.RobotOpMode;


@TeleOp(name = "Nessy Test")
public class NessyTest extends RobotOpMode {
    private NessyConfiguration config;
    private DualGamePadSteerDrive drive;
    private LiftControl liftControl;
    private ClimbControl climbControl;
    private ClawControl clawControl;
    private ArmControl armControl;

    @Override
    protected void onInit() {
        config = NessyConfiguration.newConfig(hardwareMap, telemetry);
        drive = new DualGamePadSteerDrive(this, gamepad1, config.centreLeft, config.centreRight, config.backLeft, config.backRight);
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