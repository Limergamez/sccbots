package org.firstinspires.ftc.teamcode.bessy.otos;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.shared.common.RobotOpMode;
import org.firstinspires.ftc.teamcode.shared.tasks.BaseTask;
import org.firstinspires.ftc.teamcode.shared.tasks.Task;


public class ServoTask extends BaseTask implements Task {
    private final Servo servo;
    private final double position;
    private final double tolerance;

    public ServoTask(RobotOpMode opMode, double time, Servo servo, double position, double tolerance) {
        super(opMode, time);

        this.servo = servo;
        this.position = position;
        this.tolerance = tolerance;
    }

    public ServoTask(RobotOpMode opMode, double time, Servo servo, double position) {
        this(opMode, time, servo, position, -1.0);
    }

     @Override
    public void run() {
        servo.setPosition(position);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || tolerance < 0.0 || Math.abs(servo.getPosition() - position) < tolerance;
    }

}
