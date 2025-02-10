package org.firstinspires.ftc.teamcode.opmode.Tuners;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * ServoTester uses:
 * - Right and left triggers to cycle through servos.
 * - Right joystick Y for macro control (coarse setting) of the selected servo.
 * - Bumpers for fine tuning.
 *
 * The "Arm" selection (index 0) updates both arm servos.
 */
@TeleOp(name = "Servo Tester", group = "Test")
public class servoTester extends LinearOpMode {

    // Servo hardware declarations (use your robot’s configuration names)
    private Servo armServoLeft;
    private Servo armServoRight;
    private Servo pivotServo;
    private Servo wristServo;
    private Servo clawServo;
    private Servo light;

    // Current stored positions for each servo (initialized to a safe midpoint)
    private double armPosition = 0.5;
    private double pivotPosition = 0.5;
    private double wristPosition = 0.5;
    private double clawPosition = 0.5;
    private double lightPosition = 0.5;

    // Servo selection indices:
    // 0: Arm (both), 1: Pivot, 2: Wrist, 3: Claw, 4: Light
    private int selectedServoIndex = 0;

    // Fine tuning increment/decrement step.
    private final double STEP = 0.01;

    // Previous button states to detect rising edges.
    private boolean prevRightTrigger = false;
    private boolean prevLeftTrigger = false;
    private boolean prevRightBumper = false;
    private boolean prevLeftBumper = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Map hardware (ensure these names match your configuration)
        armServoLeft  = hardwareMap.get(Servo.class, "armServoL");
        armServoRight = hardwareMap.get(Servo.class, "armServoR");
        pivotServo    = hardwareMap.get(Servo.class, "pivotServo");
        wristServo    = hardwareMap.get(Servo.class, "wristServo");
        clawServo     = hardwareMap.get(Servo.class, "clawServo");
        light         = hardwareMap.get(Servo.class, "light");

        // Set directions (one arm servo is reversed)
        armServoLeft.setDirection(Servo.Direction.FORWARD);
        armServoRight.setDirection(Servo.Direction.REVERSE);

        // Initialize servo positions
        armServoLeft.setPosition(armPosition);
        armServoRight.setPosition(armPosition);
        pivotServo.setPosition(pivotPosition);
        wristServo.setPosition(wristPosition);
        clawServo.setPosition(clawPosition);
        light.setPosition(lightPosition);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // === Servo Selection Switching via Triggers ===
            boolean rightTriggerPressed = gamepad1.right_trigger > 0.5;
            boolean leftTriggerPressed  = gamepad1.left_trigger  > 0.5;

            if (rightTriggerPressed && !prevRightTrigger) {
                selectedServoIndex = (selectedServoIndex + 1) % 5; // Cycle forward.
            }
            if (leftTriggerPressed && !prevLeftTrigger) {
                selectedServoIndex = (selectedServoIndex - 1 + 5) % 5; // Cycle backward.
            }
            prevRightTrigger = rightTriggerPressed;
            prevLeftTrigger  = leftTriggerPressed;

            // === Macro Control using Right Joystick Y ===
            // If the joystick is moved beyond a small deadzone, use its value to set a coarse position.
            if (Math.abs(gamepad1.right_stick_y) > 0.1) {
                // Map the joystick value (-1 to 1) to a servo position (0 to 1).
                // (Note: With this mapping, pushing up (–1) gives 1.0 and pushing down (+1) gives 0.0.)
                double macroVal = Range.clip(( -gamepad1.right_stick_y + 1 ) / 2.0, 0, 1);
                switch (selectedServoIndex) {
                    case 0: // Arm (both servos)
                        armPosition = macroVal;
                        armServoLeft.setPosition(armPosition);
                        armServoRight.setPosition(armPosition);
                        break;
                    case 1: // Pivot
                        pivotPosition = macroVal;
                        pivotServo.setPosition(pivotPosition);
                        break;
                    case 2: // Wrist
                        wristPosition = macroVal;
                        wristServo.setPosition(wristPosition);
                        break;
                    case 3: // Claw
                        clawPosition = macroVal;
                        clawServo.setPosition(clawPosition);
                        break;
                    case 4: // Light
                        lightPosition = macroVal;
                        light.setPosition(lightPosition);
                        break;
                }
            }

            // === Fine Tuning using Bumpers ===
            boolean rightBumperPressed = gamepad1.right_bumper;
            boolean leftBumperPressed  = gamepad1.left_bumper;

            if (rightBumperPressed && !prevRightBumper) {
                // Increment the selected servo's position.
                switch (selectedServoIndex) {
                    case 0:
                        armPosition += STEP;
                        if (armPosition > 1.0) armPosition = 1.0;
                        armServoLeft.setPosition(armPosition);
                        armServoRight.setPosition(armPosition);
                        break;
                    case 1:
                        pivotPosition += STEP;
                        if (pivotPosition > 1.0) pivotPosition = 1.0;
                        pivotServo.setPosition(pivotPosition);
                        break;
                    case 2:
                        wristPosition += STEP;
                        if (wristPosition > 1.0) wristPosition = 1.0;
                        wristServo.setPosition(wristPosition);
                        break;
                    case 3:
                        clawPosition += STEP;
                        if (clawPosition > 1.0) clawPosition = 1.0;
                        clawServo.setPosition(clawPosition);
                        break;
                    case 4:
                        lightPosition += STEP;
                        if (lightPosition > 1.0) lightPosition = 1.0;
                        light.setPosition(lightPosition);
                        break;
                }
            }
            if (leftBumperPressed && !prevLeftBumper) {
                // Decrement the selected servo's position.
                switch (selectedServoIndex) {
                    case 0:
                        armPosition -= STEP;
                        if (armPosition < 0.0) armPosition = 0.0;
                        armServoLeft.setPosition(armPosition);
                        armServoRight.setPosition(armPosition);
                        break;
                    case 1:
                        pivotPosition -= STEP;
                        if (pivotPosition < 0.0) pivotPosition = 0.0;
                        pivotServo.setPosition(pivotPosition);
                        break;
                    case 2:
                        wristPosition -= STEP;
                        if (wristPosition < 0.0) wristPosition = 0.0;
                        wristServo.setPosition(wristPosition);
                        break;
                    case 3:
                        clawPosition -= STEP;
                        if (clawPosition < 0.0) clawPosition = 0.0;
                        clawServo.setPosition(clawPosition);
                        break;
                    case 4:
                        lightPosition -= STEP;
                        if (lightPosition < 0.0) lightPosition = 0.0;
                        light.setPosition(lightPosition);
                        break;
                }
            }
            prevRightBumper = rightBumperPressed;
            prevLeftBumper  = leftBumperPressed;

            // === Telemetry Output ===
            String servoName;
            double currentPosition;
            switch (selectedServoIndex) {
                case 0:
                    servoName = "Arm (both)";
                    currentPosition = armPosition;
                    break;
                case 1:
                    servoName = "Pivot";
                    currentPosition = pivotPosition;
                    break;
                case 2:
                    servoName = "Wrist";
                    currentPosition = wristPosition;
                    break;
                case 3:
                    servoName = "Claw";
                    currentPosition = clawPosition;
                    break;
                case 4:
                    servoName = "Light";
                    currentPosition = lightPosition;
                    break;
                default:
                    servoName = "Unknown";
                    currentPosition = 0;
            }
            telemetry.addData("Selected Servo", servoName);
            telemetry.addData("Position", "%.2f", currentPosition);
            telemetry.addLine("Servo positions:")
                    .addData("Arm L", "%.2f", armServoLeft.getPosition())
                    .addData("Arm R", "%.2f", armServoRight.getPosition())
                    .addData("Pivot", "%.2f", pivotServo.getPosition())
                    .addData("Wrist", "%.2f", wristServo.getPosition())
                    .addData("Claw", "%.2f", clawServo.getPosition())
                    .addData("Light", "%.2f", light.getPosition());
            telemetry.update();

            sleep(50); // Brief pause to ease CPU load.
        }
    }
}
