package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ClawTester extends OpMode {
    private Servo clawServo;
    private double clawPosition = 0.75;

    public void init() {
        clawServo = hardwareMap.get(Servo.class, "clawServo");
    }

    public void loop() {
        if (gamepad1.square) {
            clawPosition += 0.01;
        } else if (gamepad1.circle) {
            clawPosition -= 0.01;
        }

        clawServo.setPosition(clawPosition);

        telemetry.addData("Claw Position", clawPosition);
        telemetry.update();
    }

}
