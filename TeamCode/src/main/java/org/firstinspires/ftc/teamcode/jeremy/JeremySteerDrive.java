package org.firstinspires.ftc.teamcode.jeremy;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.edubot.CharlesArmController;
import org.firstinspires.ftc.teamcode.edubot.CharlesConfiguration;
import org.firstinspires.ftc.teamcode.opencv.ColourCountVision;
import org.firstinspires.ftc.teamcode.shared.common.ClawControl;
import org.firstinspires.ftc.teamcode.shared.common.ClimbControl;
import org.firstinspires.ftc.teamcode.shared.common.DualGamePadSteerDrive;
import org.firstinspires.ftc.teamcode.shared.common.GamePadSteerDrive;
import org.firstinspires.ftc.teamcode.shared.common.LiftControl;
import org.firstinspires.ftc.teamcode.shared.common.RobotOpMode;
import org.firstinspires.ftc.teamcode.shared.common.ServoControl;
import org.opencv.core.Scalar;


@TeleOp(name = "Jeremy Steer Drive V1")
public class JeremySteerDrive extends RobotOpMode {
    private JeremyConfiguration config;
    private DualGamePadSteerDrive drive;
    private LiftControl liftControl;
    private ClimbControl climbControl;
    private ClawControl clawControl;

    @Override
    protected void onInit() {
        config = JeremyConfiguration.newConfig(hardwareMap, telemetry);
        drive = new DualGamePadSteerDrive(this, gamepad1, config.centreLeft, config.centreRight, config.backLeft, config.backRight);
        liftControl = new LiftControl(this, gamepad1, config.liftMotor);
        climbControl = new ClimbControl(this, gamepad1, config.climbMotor);
        clawControl = new ClawControl(this, gamepad1, config.clawServo);
    }

    @Override
    protected void activeLoop() throws InterruptedException {
        drive.update();
        liftControl.update();
        climbControl.update();
        clawControl.update();
        telemetry.update();
    }
}