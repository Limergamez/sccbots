package org.firstinspires.ftc.teamcode.shared.common;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ServoControl")
public class ServoControl extends OpMode {
    private Servo myServo;

    @Override
    public void init() {
        myServo = hardwareMap.get(Servo.class, "clawServo");
    }

    @Override
    public void loop() {
        // The X button should move the servo forward while the Square reverses
        if (gamepad1.b) {
            myServo.setPosition(1.0); // Full forward position
        } else if (gamepad1.a) {
            myServo.setPosition(0.0); // Full reverse position
        } //else {
            //myServo.setPosition(0.5); // iNeutral positionI dont think this is required
        }
    }
//}
