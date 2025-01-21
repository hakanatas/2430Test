package org.firstinspires.ftc.teamcode.opmode.Tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.subsystems.EndEffector;

@TeleOp(name = "Servo Tester", group = "Test")
public class servoTester extends OpMode{
    private EndEffector endEffector;
    private int servoIndex = 0;

    public void init() {
        telemetry.addLine("Initializing...");
        telemetry.update();

        endEffector = new EndEffector(hardwareMap);

        telemetry.addLine();
        telemetry.update();
    }

    public void loop() {
        Gamepad prev = gamepad1;
        if (gamepad1.left_bumper) {
            decServoIndex();
        } else if (gamepad1.right_bumper) {
            incServoIndex();
        }

        if (gamepad1.right_trigger > 0.5) {
            incServo();
        } else if (gamepad1.left_trigger > 0.5) {
            decServo();
        }

        telemetry.addData(indexToServo(), indexToPos());
        telemetry.update();


    }

    private void incServoIndex() {
        if (servoIndex == 3) {
            servoIndex = 0;
        } else {
            servoIndex++;
        }
    }

    private void decServoIndex() {
        if (servoIndex == 0) {
            servoIndex = 3;
        } else {
            servoIndex--;
        }
    }

    private String indexToServo() {
        switch (servoIndex) {
            case 0:
                return "armServo";
            case 1:
                return "pivotServo";
            case 2:
                return "wristServo";
            case 3:
                return "clawServo";
            default:
                return "armServo";
        }
    }

    private double indexToPos() {
        switch (servoIndex) {
            case 0:
                return endEffector.getArmPosition();
            case 1:
                return endEffector.getPivotPosition();
            case 2:
                return endEffector.getWristPosition();
            case 3:
                return endEffector.getClawPosition();
            default:
                return endEffector.getArmPosition();
        }
    }

    private void incServo() {
        switch (servoIndex) {
            case 0:
                endEffector.setArmPosition(endEffector.getArmPosition() + 0.005);
                break;
            case 1:
                endEffector.setPivotPosition(endEffector.getPivotPosition() + 0.005);
                break;
            case 2:
                endEffector.setWristPosition(endEffector.getWristPosition() + 0.005);
                break;
            case 3:
                endEffector.setClawPosition(endEffector.getClawPosition() + 0.005);
                break;
        }
    }

    private void decServo() {
        switch (servoIndex) {
            case 0:
                endEffector.setArmPosition(endEffector.getArmPosition() - 0.005);
                break;
            case 1:
                endEffector.setPivotPosition(endEffector.getPivotPosition() - 0.005);
                break;
            case 2:
                endEffector.setWristPosition(endEffector.getWristPosition() - 0.005);
                break;
            case 3:
                endEffector.setClawPosition(endEffector.getClawPosition() - 0.005);
                break;
        }
    }
}
