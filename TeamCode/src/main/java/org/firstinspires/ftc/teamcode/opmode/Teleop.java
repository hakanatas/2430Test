package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.teamcode.config.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.config.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.config.subsystems.EndEffector;
import org.firstinspires.ftc.robotcontroller.internal.GoBildaPinpointDriver;

@TeleOp(name = "Teleop", group = "Teleop")
public class Teleop extends OpMode {

    // Hardware references
    private MecanumDrive drive;
    private Deposit slides;
    private EndEffector endEffector;
    private GoBildaPinpointDriver pinpoint;

    // Gamepad snapshots
    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad previousGamepad1 = new Gamepad();

    // State machine variables
    private int intakeState = -1;
    private int depositState = -1;
    private int phase = 0; // used in your multi-phase intake logic
    private ElapsedTime specIntakeTimer = new ElapsedTime();

    private ElapsedTime liftResetTimer = new ElapsedTime();
    private boolean isLiftResetting = false;

    @Override
    public void init() {
        // Set up bulk caching
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Initialize subsystems
        drive = new MecanumDrive();
        slides = new Deposit(hardwareMap, telemetry, false);
        endEffector = new EndEffector(hardwareMap);

        drive.init(hardwareMap);
        endEffector.setIdlePosition(); // set the end effector to idle position

        // Pinpoint driver initialization
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        configurePinpoint();

        // Reset your timers, etc.
        specIntakeTimer.reset();

        telemetry.addLine("Ready!");
        telemetry.update();
    }




    @Override
    public void loop() {
        //------------------------------------------
        // 1) Copy previous -> current gamepad
        //------------------------------------------
        // First, remember what the previous state was
        previousGamepad1.copy(currentGamepad1);
        // Then copy the new hardware state into current
        currentGamepad1.copy(gamepad1);

        //------------------------------------------
        // 2) Drive / IMU Controls
        //------------------------------------------
        double forward = -currentGamepad1.left_stick_y;
        double strafe  =  currentGamepad1.left_stick_x;
        double rotate  =  currentGamepad1.right_stick_x;

        Pose2D currentPose = driveFieldRelative(forward, strafe, rotate);

        // If x is pressed, reset IMU
        if (currentGamepad1.square) {
            pinpoint.resetPosAndIMU();
            pinpoint.recalibrateIMU();
        }

        if (currentGamepad1.triangle) {
            slides.manualControl = true;
            slides.pidfActive = false;
            slides.positivePower = true;
        } else if (currentGamepad1.cross) {
            slides.manualControl = true;
            slides.positivePower = false;
        } else {
            slides.setSlideTarget(slides.liftPos);
            slides.manualControl = false;
            slides.pidfActive = true;
        }
        // If y is pressed, recalibrate IMU

        // Circle button logic
        boolean circleJustPressed = currentGamepad1.circle && !previousGamepad1.circle;
        boolean circleReleased = !currentGamepad1.circle && previousGamepad1.circle;
        if (circleJustPressed) {
            endEffector.setIdlePosition();
            intakeState = -1;
            depositState = -1;
            slides.pidfActive = false;
            slides.setPivotTarget(90);
            slides.setSlideTarget(0);
        }


        //------------------------------------------
        // 3) Rising Edge Detection for Bumpers/Triggers
        //------------------------------------------
        boolean leftBumperJustPressed =
                (currentGamepad1.left_bumper && !previousGamepad1.left_bumper);
        boolean rightBumperJustPressed =
                (currentGamepad1.right_bumper && !previousGamepad1.right_bumper);

        boolean leftTriggerJustPressed =
                (currentGamepad1.left_trigger > 0.5f) && (previousGamepad1.left_trigger <= 0.5f);
        boolean rightTriggerJustPressed =
                (currentGamepad1.right_trigger > 0.5f) && (previousGamepad1.right_trigger <= 0.5f);



//        if (circleJustPressed) {
//            // Stop PIDF control
//            slides.stopPIDF();
//            // Set pivot target and idle position
//            slides.setPivotTarget(90);
//            endEffector.setIdlePosition();
//            // Move the lift downward at -0.5 power
//            slides.setLiftPower(-0.5);
//            isLiftResetting = false; // Reset flag
//        }
//
//        if (currentGamepad1.circle) {
//            // Continue holding lift power while the circle is pressed
//            slides.setLiftPower(-0.5);
//        }
//
//        if (circleReleased) {
//            // Stop lift motor and reset encoder
//            slides.setLiftPower(0);
//            slides.resetLiftEncoder();
//
//            // Start the timer for the delay
//            liftResetTimer.reset();
//            isLiftResetting = true;
//        }
//
//        // Check if the delay has elapsed
//        if (isLiftResetting && liftResetTimer.milliseconds() >= 150) {
//            slides.startPIDF();
//            isLiftResetting = false; // Reset the flag
//        }



        //------------------------------------------
        // 4) Spec Intake (left bumper +/ left trigger -)
        //------------------------------------------
        if (leftBumperJustPressed) {
            intakeState = (intakeState + 1) % 4; // cycle 0..3
        }
        if (leftTriggerJustPressed) {
            intakeState = (intakeState - 1) % 4; // cycle 0..3
        }

        //------------------------------------------
        // 5) Spec Deposit (right bumper +/ right trigger -)
        //------------------------------------------
        if (rightBumperJustPressed) {
            depositState = (depositState + 1) % 5; // cycle 0..4
        }
        if (rightTriggerJustPressed) {
            depositState = (depositState - 1) % 5; // cycle 0..4
        }

        // If both states are active, reset them (example logic — adjust to your liking)
        if (intakeState != -1 && depositState != -1) {
            intakeState = -1;
            depositState = -1;
        }

        //------------------------------------------
        // 6) DPAD Controls for the Claw & Pivot/Wrist
        //------------------------------------------
        boolean dpadLeftJustPressed =
                (currentGamepad1.dpad_left && !previousGamepad1.dpad_left);
        boolean dpadRightJustPressed =
                (currentGamepad1.dpad_right && !previousGamepad1.dpad_right);
        boolean dpadUpJustPressed =
                (currentGamepad1.dpad_up && !previousGamepad1.dpad_up);
        boolean dpadDownJustPressed =
                (currentGamepad1.dpad_down && !previousGamepad1.dpad_down);

        // --- Claw Open/Close ---
        if (dpadLeftJustPressed) {
            endEffector.openClaw();
        }
        if (dpadRightJustPressed) {
            endEffector.closeClaw();
        }

        // --- Pivot or Wrist Control ---
        if (slides.pivotTarget > 45) {
            // Control pivot
            if (currentGamepad1.dpad_up) {
                endEffector.incrementPivotPosition(0.02);
            }
            if (currentGamepad1.dpad_down) {
                endEffector.decrementPivotPosition(0.02);
            }
        } else {
            // Control wrist
            if (currentGamepad1.dpad_up) {
                endEffector.incrementWristPosition(0.02);
            }
            if (currentGamepad1.dpad_down) {
                endEffector.decrementWristPosition(0.02);
            }
        }

        //------------------------------------------
        // 7) Intake State Machine
        //------------------------------------------
        switch (intakeState) {
            case 0:
                // pivot=0, slide=50, idle pos, open claw
                slides.setPivotTarget(0);
                slides.setSlideTarget(50);
                endEffector.setIdlePosition();
                endEffector.openClaw();
                specIntakeTimer.reset();
                phase = 0;
                break;

            case 1:
                // move lift to 400
                slides.setSlideTarget(400);
                if (slides.liftPos > 380) {
                    endEffector.setPreSubPickupPosition();
                }
                specIntakeTimer.reset();
                phase = 0;
                break;
            case 2:

                if (currentGamepad1.triangle) {
                    slides.manualControl = true;
                    slides.pidfActive = false;
                    slides.positivePower = true;
                } else if (currentGamepad1.cross) {
                    slides.manualControl = true;
                    slides.positivePower = false;
                } else {
                    slides.setSlideTarget(slides.liftPos);
                    slides.manualControl = false;
                    slides.pidfActive = true;
                }
                // multi-phase logic
                switch (phase) {
                    case 0: // subPickup
                        if (specIntakeTimer.milliseconds() == 0) {
                            specIntakeTimer.reset();
                        }
                        endEffector.setSubPickupPosition();
                        endEffector.closeClaw();

                        if (specIntakeTimer.milliseconds() >= 150) {
                            phase = 1;
                            specIntakeTimer.reset();
                        }
                        break;
                    case 1: // deposit
                        endEffector.setObsDepositPosition();
                        slides.setPivotTarget(12);
                        slides.setSlideTarget(50);
                        break;
                }
                break;

            case 3:
                slides.setPivotTarget(90);
                specIntakeTimer.reset();
                phase = 0;
                break;

            default:
                specIntakeTimer.reset();
                phase = 0;
                break;
        }

        //------------------------------------------
        // 8) Deposit State Machine
        //------------------------------------------
        switch (depositState) {
            case 0:
                slides.setPivotTarget(90);
                slides.setSlideTarget(0);
                endEffector.setWallIntakePositionAlt();
                endEffector.openClaw();
                break;

            case 1:
                endEffector.closeClaw();
                break;
            case 2:
                endEffector.setSpecScore();
                break;

            case 3:
                slides.setSlideTarget(475);
                break;

            case 4:
                slides.setSlideTarget(230);
                if (slides.liftPos < 250) {
                    endEffector.openClaw();
                    depositState = 4;
                }
                break;
            default:
                // do nothing
                break;
        }

        // Update deposit mechanics
        slides.update();

        //------------------------------------------
        // 9) Telemetry
        //------------------------------------------
        String data = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                currentPose.getX(DistanceUnit.INCH),
                currentPose.getY(DistanceUnit.INCH),
                currentPose.getHeading(AngleUnit.DEGREES)
        );
        telemetry.addData("Position", data);
        telemetry.addData("Status", pinpoint.getDeviceStatus());
        telemetry.addData("Pinpoint Frequency", pinpoint.getFrequency());
        telemetry.addData("Intake State", intakeState);
        telemetry.addData("Deposit State", depositState);
        telemetry.addData("At set point", slides.slidesReached);
        telemetry.addData("Lift Pos", slides.liftPos);
        telemetry.addData("Lift Target", slides.slidePIDF.getSetPoint());
        telemetry.addData("Error", slides.slidePIDF.getPositionError());
        telemetry.update();
    }

    // ------------------------------------------------
    // HELPER: Configure the GoBilda Pinpoint device
    // ------------------------------------------------
    private void configurePinpoint() {
        pinpoint.setOffsets(153.22873, -68.86620);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpoint.resetPosAndIMU();
    }

    // ------------------------------------------------
    // HELPER: Field-relative drive
    // ------------------------------------------------
    private Pose2D driveFieldRelative(double forward, double right, double rotate) {
        pinpoint.update();
        Pose2D pos = pinpoint.getPosition();  // Current position

        double robotAngle = Math.toRadians(pos.getHeading(AngleUnit.DEGREES));
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(forward, right);
        theta = org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
                .normalizeRadians(theta - robotAngle);

        double newForward = r * Math.sin(theta);
        double newRight   = r * Math.cos(theta);

        drive.drive(newForward, newRight, rotate);
        return pos;
    }
}