package org.firstinspires.ftc.teamcode.bessy.otos;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.shared.common.RobotOpMode;
import org.firstinspires.ftc.teamcode.shared.tasks.BaseTask;
import org.firstinspires.ftc.teamcode.shared.tasks.Task;


public class MotorTask extends BaseTask implements Task {
    private final DcMotor dcMotor;
    private final double power;

    public MotorTask(RobotOpMode opMode, double time, DcMotor dcMotor, double power) {
        super(opMode, time);

        this.dcMotor = dcMotor;
        this.power = power;
    }

    public MotorTask(RobotOpMode opMode, double time, DcMotor dcMotor, double power, boolean brake) {
        this(opMode, time, dcMotor, power);
        if (brake) {
            dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    @Override
    public void init() {
        dcMotor.setPower(power);
    }

     @Override
    public void run() {
    }

    @Override
    public boolean isFinished() {
        if (super.isFinished()) {
            dcMotor.setPower(0.0);
            return true;
        }
        return false;
    }

}
