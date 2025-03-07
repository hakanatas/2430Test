package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "aaaaaaaaOne Motor Test")
public class OneMotor extends OpMode {
  private DcMotor fl;


    double leftStickYVal;
    double leftStickXVal;
    double rightStickXVal;
    double rightStickYVal;

    double frontLeftSpeed;
    double frontRightSpeed;
    double rearLeftSpeed;
    double rearRightSpeed;

    double powerThreshold = 0.35;


    //   private Arm arm;
    // private EndEffector endEffector;



    @Override
    public void init() {
        telemetry.addLine("Initializing...");
        telemetry.update();

        // encoder = hardwareMap.get(AnalogInput.class, "enc");
        fl = hardwareMap.get(DcMotorEx.class, "fl");



//        arm = new Arm(hardwareMap);
//        endEffector = new EndEffector(hardwareMap);
//
//        sparkfunOTOS = hardwareMap.get(SparkFunOTOS.class, HWValues.OTOS);
//        configureOTOS();
        telemetry.addLine("Ready!");
        telemetry.update();
    }

    /**
     * This runs the OpMode. This is only drive control with Pedro Pathing live centripetal force
     * correction.
     */
    @Override
    public void loop() {




        drive();
       telemetry();

//
//


        // Telemetry
    }


    public void drive() {




                leftStickYVal = gamepad1.left_stick_y;
                leftStickYVal = Range.clip(leftStickYVal, -1, 1);
                leftStickXVal = -gamepad1.left_stick_x;
                leftStickXVal = Range.clip(leftStickXVal, -1, 1);


                frontLeftSpeed = leftStickYVal + leftStickXVal + rightStickXVal;
                frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);

                frontRightSpeed = leftStickYVal - leftStickXVal - rightStickXVal;
                frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);

                rearLeftSpeed = leftStickYVal - leftStickXVal + rightStickXVal;
                rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);

                rearRightSpeed = leftStickYVal + leftStickXVal - rightStickXVal;
                rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);

                if (frontLeftSpeed <= powerThreshold && frontLeftSpeed >= -powerThreshold) {
                    frontLeftSpeed = 0;
                    fl.setPower(frontLeftSpeed);
                } else {
                    fl.setPower(frontLeftSpeed);
                }


        }


    public void telemetry(){
        telemetry.addData("fl power", fl.getPower());

        telemetry.update();

    }



}
