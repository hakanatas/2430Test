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
    private ElapsedTime specDepoTimer = new ElapsedTime();

    // Example bounding box, in inches, around the origin or wherever you’ve chosen:
    private static final double X_MIN = -4;  // left boundary
    private static final double X_MAX =  4;  // right boundary
    private static final double Y_MIN = -4;  // bottom boundary
    private static final double Y_MAX =  4;  // top boundary
    private static final double SMOOTH_ZONE = 4.0;
    private static final boolean demo = true;


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
        endEffector.setLight(0.5);
        EndEffector.override = false;


        drive.init(hardwareMap);
        // Pinpoint driver initialization
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        configurePinpoint();

        // Reset your timers, etc.
        specIntakeTimer.reset();
        specDepoTimer.reset();

        telemetry.addLine("Ready!");
        telemetry.update();
    }






    @Override
    public void loop() {
        endEffector.update();

        if (EndEffector.override) {
            endEffector.setLight(0);
        }
        else if (endEffector.pin0() && endEffector.pin1()) {
            endEffector.setLight(0.388);
        } else if (endEffector.pin0()) {
            endEffector.setLight(0.611);
        } else if (endEffector.pin1()) {
            endEffector.setLight(0.29);
        } else {
            endEffector.setLight(0.5);
        }

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
        }
        // If y is pressed, recalibrate IMU
        if (currentGamepad1.triangle) {
            pinpoint.recalibrateIMU();
        }
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

        boolean shareJustPressed = (currentGamepad1.share && !previousGamepad1.share);
        if (shareJustPressed) {
            EndEffector.override = !EndEffector.override;
        }


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
            depositState = 0; // cycle 0..4
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
                slides.setPivotTarget(10);
                slides.setSlideTarget(0);
                endEffector.setIdlePosition();
                endEffector.openClaw();
                specIntakeTimer.reset();
                phase = 0;
                break;

            case 1:
                // move lift to 400
                slides.setSlideTarget(450);
                if (slides.liftPos > 380) {
                    // OPENING CLAW COULD BE DANGEROUS HERE
                    endEffector.openClaw();
                    endEffector.setPreSubPickupPosition();
                } else if (slides.slideTarget > 300) {
                    endEffector.setIdlePosition();
                    endEffector.openClaw();
                }

                specIntakeTimer.reset();
                phase = 0;
                break;

            case 2:
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
                        endEffector.setSafeIdle();
                        slides.setPivotTarget(10);
                        slides.setSlideTarget(0);
                        if (specIntakeTimer.milliseconds() >= 200) {
                            if (!endEffector.pin0() && !endEffector.pin1()) {
                                intakeState = 1;
                            }
                        }
                        break;
                }
                break;

            case 3:
                slides.setPivotTarget(90);
                endEffector.setObsDepositPosition();
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
                specDepoTimer.reset();
                break;

            case 1:
                if (!endEffector.pin0() && !endEffector.pin1()) {
                    depositState = 0;
                    specDepoTimer.reset();
                    break;
                }
                endEffector.closeClaw();
                if (specDepoTimer.milliseconds() > 300) {
                    depositState = 2;
                    specDepoTimer.reset();
                }
                break;
            case 2:
                endEffector.setSpecScore();
                specDepoTimer.reset();
                break;

            case 3:
                slides.setSlideTarget(475);
                specDepoTimer.reset();
                break;

            case 4:
                slides.setSlideTarget(210);
                if (slides.liftPos < 220) {
                    endEffector.openClaw();
                }
                if (specDepoTimer.milliseconds() > 250) {
                    depositState = 0;
                    specDepoTimer.reset();
                }
                break;
            default:
                specDepoTimer.reset();
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
//        telemetry.addData("Intake State", intakeState);
//        telemetry.addData("Deposit State", depositState);
//        telemetry.addData("At set point", slides.slidesReached);
        telemetry.addData("Lift Pos", slides.liftPos);
        telemetry.addData("Lift Target", slides.slidePIDF.getSetPoint());
        telemetry.addData("Pivot Pos", slides.pivotPos);
        telemetry.addData("Pivot Target", slides.pivotTarget);

//        telemetry.addData("Error", slides.slidePIDF.getPositionError());
//        telemetry.addData("Pin0", endEffector.pin0());
//        telemetry.addData("Pin1", endEffector.pin1());
        telemetry.update();
    }

    // ------------------------------------------------
    // HELPER: Configure the GoBilda Pinpoint device
    // ------------------------------------------------
    private void configurePinpoint() {
        pinpoint.setOffsets(6.03262717 * 25.4,2.71126772 * 25.4);
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
//    private Pose2D driveFieldRelative(double forward, double right, double rotate) {
//        pinpoint.update();
//        Pose2D pos = pinpoint.getPosition();  // Current position
//
//        double robotAngle = Math.toRadians(pos.getHeading(AngleUnit.DEGREES));
//        double theta = Math.atan2(forward, right);
//        double r = Math.hypot(forward, right);
//        theta = org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
//                .normalizeRadians(theta - robotAngle);
//
//        double newForward = r * Math.sin(theta);
//        double newRight   = r * Math.cos(theta);
//
//        if (demo) {
//
//        }
//
//        drive.drive(newForward, newRight, rotate);
//        return pos;
//    }



    private Pose2D driveFieldRelative(double forward, double right, double rotate) {
        pinpoint.update();
        Pose2D pos = pinpoint.getPosition();  // Current position

        double robotAngle = Math.toRadians(pos.getHeading(AngleUnit.DEGREES));

        // Convert driver input into field-centric motion
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(forward, right);
        theta = AngleUnit.normalizeRadians(theta - robotAngle);

        // These are the "robot-centric" forward & strafe after field correction
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        double xPos = pos.getX(DistanceUnit.INCH);
        double yPos = pos.getY(DistanceUnit.INCH);


        if (demo) {

        }

        telemetry.addData("xPos", xPos);
        telemetry.addData("yPos", yPos);
        telemetry.addData("newForward", newForward);
        telemetry.addData("newRight", newRight);
        // Now drive with our final scaled inputs
        drive.drive(newForward, newRight, rotate);

        return pos;
    }

    // Simple clamp helper
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}