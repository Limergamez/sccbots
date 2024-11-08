package org.firstinspires.ftc.teamcode.edubot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shared.common.ButtonControl;
import org.firstinspires.ftc.teamcode.shared.common.RobotComponent;
import org.firstinspires.ftc.teamcode.shared.common.RobotOpMode;


/**
 * Operation to assist with Gamepad actions on DCMotors
 */
public class CharlesArmController extends RobotComponent {

    private final ButtonControl armUpButtonControl;
    private final ButtonControl armDownButtonControl;
    private final ButtonControl armClawOpenControl;
    private final ButtonControl armClawCloseControl;

    private final DcMotor armMotor;
    private final Servo clawServo;
    private final Gamepad gamepad;
    private final TouchSensor stopSensor;
    private final float motorPower;
    private final Telemetry.Item item;
    private final Telemetry.Item item2;
    private boolean showtelemetry = false;

    /**
     * Constructor for operation.  Telemetry enabled by default.
     *
     * @param opMode
     * @param gamepad              Gamepad
     * @param config               EduBotConfiguration
     * @param power                power to apply when using gamepad buttons
     * @param showTelemetry        display the power values on the telemetry
     */
    public CharlesArmController(RobotOpMode opMode, Gamepad gamepad, CharlesConfiguration config,
                                float power, boolean showTelemetry) {
        super(opMode);

        this.gamepad = gamepad;
        this.armMotor = config.armMotor;
        this.clawServo = config.armServo;
        this.armUpButtonControl = ButtonControl.A;
        this.armDownButtonControl = ButtonControl.B;
        this.armClawOpenControl = ButtonControl.X;
        this.armClawCloseControl = ButtonControl.Y;
        this.motorPower = power;
        this.stopSensor = config.touchSensor;

        if (showTelemetry) {
            item = opMode.telemetry.addData("Arm " + armUpButtonControl.name() + "/" + armDownButtonControl.name(), new Func<Double>() {
                @Override
                public Double value() {
                    return armMotor.getPower();
                }
            });
            item.setRetained(true);

            item2 = opMode.telemetry.addData("Claw " + armClawOpenControl.name() + "/" + armClawCloseControl.name(), new Func<Double>() {
                @Override
                public Double value() {
                    return clawServo.getPosition();
                }
            });
            item2.setRetained(true);
        } else {
            item = null;
            item2 = null;
        }

        clawServo.setPosition(0.5);
    }


    public CharlesArmController(RobotOpMode opMode, Gamepad gamepad, CharlesConfiguration config,
                                float power) {
        this(opMode, gamepad, config,  power, true);
    }

    /**
     * Update motors with latest gamepad state
     */
    public void update() {
        // Arm
        if (buttonPressed(gamepad, armUpButtonControl)) {
            armMotor.setPower(motorPower);
        } else  if (stopSensor != null && stopSensor.isPressed()) {
            armMotor.setPower(0.0);
        } else if (buttonPressed(gamepad, armDownButtonControl)) {
            armMotor.setPower(-motorPower);
        } else {
            armMotor.setPower(0.0);
        }

        // Claw
        if (buttonPressed(gamepad, armClawOpenControl)) {
            clawServo.setPosition(0.1);
        } else if (buttonPressed(gamepad, armClawCloseControl)) {
            clawServo.setPosition(0.8);
        }
    }
}
