package org.firstinspires.ftc.teamcode.bessy;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.shared.common.ArmControl;
import org.firstinspires.ftc.teamcode.shared.common.ClawControl;
import org.firstinspires.ftc.teamcode.shared.common.ClimbControl;
import org.firstinspires.ftc.teamcode.shared.common.LiftControl;
import org.firstinspires.ftc.teamcode.shared.common.rodHarvester;
import org.firstinspires.ftc.teamcode.shared.common.spinHarvester;
import org.firstinspires.ftc.teamcode.shared.common.rotateHarvester;
import org.firstinspires.ftc.teamcode.shared.common.RobotOpMode;
import org.firstinspires.ftc.teamcode.tessy.DualGamePadSteerDriveMecanum;
import org.firstinspires.ftc.teamcode.tessy.TessyConfiguration;



@TeleOp(name = "Bessy Steer Drive V2")
public class BessySteerDriveMecanum extends RobotOpMode {
    private TessyConfiguration config;
    private DualGamePadSteerDriveMecanum drive;
    private LiftControl liftControl;
    private ClimbControl climbControl;
    private ClawControl clawControl;
    private ArmControl armControl;
    private rodHarvester rodHarvester;
    private spinHarvester spinHarvester;
    private rotateHarvester rotateHarvester;

    @Override
    protected void onInit() {
        config = BessyConfiguration.newConfig(hardwareMap, telemetry);

        drive = new DualGamePadSteerDriveMecanum(
                this,
                gamepad1,
                config.leftBackMotor,
                config.rightBackMotor,
                config.leftFrontMotor,
                config.rightFrontMotor
        );

        liftControl = new LiftControl(this, gamepad2, config.liftMotor);
        climbControl = new ClimbControl(this, gamepad2, config.climbMotor);
        clawControl = new ClawControl(this, gamepad2, config.clawServo);
        armControl = new ArmControl(this, gamepad2, config.armMotor);

        rodHarvester = new rodHarvester(this, gamepad1, config.rodHarvesterServo);
        spinHarvester = new spinHarvester(this, gamepad1, config.spinHarvesterServo);
        rotateHarvester = new rotateHarvester(this, gamepad1, config.rotateHarvesterServo);
    }

    @Override
    protected void activeLoop() throws InterruptedException {
        drive.update();
        liftControl.update();
        climbControl.update();
        clawControl.update();
        armControl.update();
        rodHarvester.update();
        spinHarvester.update();
        rotateHarvester.update();
        telemetry.update();
    }
}
