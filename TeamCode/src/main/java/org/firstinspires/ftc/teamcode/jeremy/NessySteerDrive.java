package org.firstinspires.ftc.teamcode.jeremy;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.shared.common.ArmControl;
import org.firstinspires.ftc.teamcode.shared.common.ClawControl;
import org.firstinspires.ftc.teamcode.shared.common.ClimbControl;
import org.firstinspires.ftc.teamcode.shared.common.DualGamePadSteerDrive;
import org.firstinspires.ftc.teamcode.shared.common.LiftControl;
import org.firstinspires.ftc.teamcode.shared.common.RobotOpMode;
import org.firstinspires.ftc.teamcode.shared.common.SuperSlowTurnControl;


@TeleOp(name = "Nessy Steer Drive V2")
public class NessySteerDrive extends RobotOpMode {
    private NessyConfiguration config;
    private DualGamePadSteerDrive drive;
    private LiftControl liftControl;
    private ClimbControl climbControl;
    private ClawControl clawControl;
    private ArmControl armControl;
    private SuperSlowTurnControl superSlowTurnControl;

    @Override
    protected void onInit() {
        config = NessyConfiguration.newConfig(hardwareMap, telemetry);
        drive = new DualGamePadSteerDrive(this, gamepad1, config.centreLeft, config.centreRight, config.backLeft, config.backRight);
        liftControl = new LiftControl(this, gamepad2, config.liftMotor);
        climbControl = new ClimbControl(this, gamepad2, config.climbMotor);
        clawControl = new ClawControl(this, gamepad2, config.clawServo);
        armControl = new ArmControl(this, gamepad2, config.armMotor);
        superSlowTurnControl = new SuperSlowTurnControl(this, gamepad2, config.centreLeft, config.centreRight, config.backLeft, config.backRight);
    }

    @Override
    protected void activeLoop() throws InterruptedException {
        drive.update();
        liftControl.update();
        climbControl.update();
        clawControl.update();
        armControl.update();
        superSlowTurnControl.update();
        telemetry.update();
    }
}