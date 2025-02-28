package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;

import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;

public class DepositSubsytem extends Subsystem {

    public static final DepositSubsytem INSTANCE = new DepositSubsytem();
    private DepositSubsytem() {};

    public enum DepositState {    // The current "state" or position of the elevator
        HOME(0, 90),
        SPEC_INTAKE(0, 121),
        SPEC_SCORE(450, 121),
        SPEC_SAFE(430, 121);
        public final int liftTarget;
        public final int pivotTarget;
        DepositState(int liftTarget, int pivotTarget) {
            this.liftTarget = liftTarget;
            this.pivotTarget = pivotTarget;
        }
    }

    public MotorEx rightMotor;
    public String rightMotorName = "rightLift";
    public MotorEx leftMotor;
    public String leftMotorName = "leftLift";
    public TouchSensor slideLimit;
    public String slideLimitName = "slide_limit";
    public MotorEx pivotMotor;
    public String pivotMotorName = "pivot";
    public AnalogInput pivotEncoder;
    public String pivotEncoderName = "pivot_enc";
    public PIDFController liftController = new PIDFController(0.1, 0.1, 0.1);
    public PIDFController pivotController = new PIDFController(0.1, 0.1, 0.1);
    public double pivotPos;
    public double liftPos;
    public boolean pidfActive = true;
    public DepositState depositState;


    @Override
    public void initialize() {
        depositState = DepositState.HOME;
        rightMotor = new MotorEx(rightMotorName);
        leftMotor = new MotorEx(leftMotorName);
        slideLimit = OpModeData.INSTANCE.getHardwareMap().get(TouchSensor.class, slideLimitName);

        pivotMotor = new MotorEx(pivotMotorName);
        pivotEncoder = OpModeData.INSTANCE.getHardwareMap().get(AnalogInput.class, pivotEncoderName);

        leftMotor.reverse();
        rightMotor.resetEncoder();
        leftMotor.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pivotMotor.reverse();
        pivotMotor.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftController.setSetPointTolerance(15);
        pivotController.setSetPointTolerance(1);
    }

    @Override
    public void periodic() {
        liftPos = liftPos();
        pivotPos = pivotPos();
        liftController.setTarget(depositState.liftTarget);
        pivotController.setTarget(depositState.pivotTarget);
        double liftPower = liftController.calculate(liftPos);
        double pivotPower = pivotController.calculate(pivotPos);


    }

    private int liftPos() {
        return Math.round((float) rightMotor.getCurrentPosition() / 42) * -1;
    }

    private int pivotPos() {
        int pos = (int) (Math.round(pivotEncoder.getVoltage() / 3.2 * 360)) % 360 - 171;

        if (pos < 0) {
            pos += 360;
        }
        if (pos > 345) {
            pos = 0;
        }
        return pos;
    }


}
