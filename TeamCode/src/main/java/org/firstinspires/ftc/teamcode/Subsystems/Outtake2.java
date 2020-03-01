package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

@Config
public class Outtake2 implements Subsystem {
    ExpansionHubMotor lift;
    private int liftPosition;
    public static final double LIFT_ITERATION = 4.0;
    public static final double PULLEY_RADIUS = 2.0/2;
    public static final double TICKS_PER_REV = 537.6;
    private PIDFController liftPositionController;
    public static PIDCoefficients LiftPIDCoefficients = new PIDCoefficients(0.006,0.0001,0);
    public static double liftKG = 0.01;

    ExpansionHubServo wristLeft;
    ExpansionHubServo wristRight;
    public static double wristDeployPosition = 0.6;
    public static double wristGrabPosition = 0.17;
    public static double wristIdlePosition = 0.25;
    public static double wristLiftPosition = 0.25;
    public static double wristDelay = 0.5;
    public static double wristDropDelay = 1;

    ExpansionHubServo grabber;
    public static double grabberArmPosition = 1;
    public static double grabberDisarmPosition = 0.6;
    public static double grabberCapPosition = 0.17;

    RevTouchSensor limit;

    public enum Mode {
        OPEN_LOOP,
        RUN_TO_POSITION,
        RESET
    }

    public enum WristPosition {
        GRAB,
        LIFT,
        DEPLOY,
        IDLE
    }

    public enum LiftState {
        RAISED,
        LOWERED
    }

    private Mode mode;
    private WristPosition wristPosition;
    private LiftState liftState;

    NanoClock clock;
    double startTimestamp;



    public Outtake2(HardwareMap hardwareMap) {
        lift = hardwareMap.get(ExpansionHubMotor.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //lift.setDirection(DcMotorSimple.Direction.REVERSE);

        liftPositionController = new PIDFController(LiftPIDCoefficients,0,0,0, (x) -> liftKG);
        liftPositionController.setOutputBounds(-1,1);

        wristLeft = hardwareMap.get(ExpansionHubServo.class, "wristLeft");
        wristRight = hardwareMap.get(ExpansionHubServo.class, "wristRight");
        wristLeft.setPwmRange(new PwmControl.PwmRange(500,2500));
        wristRight.setPwmRange(new PwmControl.PwmRange(500,2500));
        wristLeft.setDirection(Servo.Direction.REVERSE);

        grabber = hardwareMap.get(ExpansionHubServo.class, "grabber");

        limit = hardwareMap.get(RevTouchSensor.class, "limit");

        mode = Mode.OPEN_LOOP;
        wristPosition = WristPosition.IDLE;
        liftState = LiftState.LOWERED;

        clock = NanoClock.system();
        liftPosition = 0;
    }

    public void setLiftPower(double power) {
        if (mode == Mode.OPEN_LOOP) {
            if (power == 0 && liftState == LiftState.RAISED) lift.setPower(0.01);
            else lift.setPower(power);
        }
    }

    public void setMode(Mode mode) {
        this.mode = mode;
    }

    public Mode getMode() {
        return mode;
    }

    public void setWristPosition(double position) {
        wristLeft.setPosition(position);
        wristRight.setPosition(position);
    }

    public WristPosition getWristPosition() {
        return wristPosition;
    }

    public void armGrabber() {
        grabber.setPosition(grabberArmPosition);
    }

    public void disarmGrabber() {
        grabber.setPosition(grabberDisarmPosition);
    }

    public void setGrabberPosition (double position) {
        grabber.setPosition(position);
    }

    public void raiseLift() {
        mode = Mode.RUN_TO_POSITION;
        if (liftState == LiftState.RAISED)liftPositionController.setTargetPosition(-(liftPosition - 1) * encoderInchesToTicks(LIFT_ITERATION) - encoderInchesToTicks(1));
    }

    public void lowerLift() {
        mode = Mode.RUN_TO_POSITION;
        liftPositionController.setTargetPosition(0);
    }

    public void liftPositionUp() {
        if (liftPosition != 7) liftPosition ++;
        raiseLift();
    }

    public void liftPositionDown() {
        if (liftPosition != 1) liftPosition --;
        raiseLift();
    }

    public void cap() {
        if (wristPosition == WristPosition.DEPLOY)
        grabber.setPosition(grabberCapPosition);
    }

    public void cycleWrist() {
        switch (wristPosition) {
            case IDLE:
                startTimestamp = clock.seconds();
                wristPosition = WristPosition.GRAB;
                break;
            case GRAB:
                //liftPositionUp();
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                liftPosition ++;
                liftState = LiftState.RAISED;
                //mode = Mode.RUN_TO_POSITION;
                liftPositionController.reset();
                liftPositionController.setTargetPosition(-(liftPosition - 1) * encoderInchesToTicks(LIFT_ITERATION) - encoderInchesToTicks(1));

                setWristPosition(wristLiftPosition);
                wristPosition = WristPosition.LIFT;
                break;
            case LIFT:
                mode = Mode.OPEN_LOOP;
                wristPosition = WristPosition.DEPLOY;
                break;
            case DEPLOY:
                liftPositionController.reset();
                //mode = Mode.RUN_TO_POSITION;
                liftPositionController.setTargetPosition(-(liftPosition - 1) * encoderInchesToTicks(LIFT_ITERATION) - encoderInchesToTicks(2));
                startTimestamp = clock.seconds();
                setWristPosition(wristIdlePosition);
                wristPosition = WristPosition.IDLE;
                break;
        }
    }

    public void deploy() {
        wristPosition = WristPosition.DEPLOY;
    }

    public double getLiftPosition() {
        return encoderTicksToInches(lift.getCurrentPosition());
    }

    public static double encoderTicksToInches (double ticks) {
        return PULLEY_RADIUS * 2 * Math.PI * ticks / TICKS_PER_REV;
    }

    public static int encoderInchesToTicks (double inches) {
        return (int) (inches * TICKS_PER_REV / PULLEY_RADIUS / 2 / Math.PI);
    }

    @Override
    public void update() {
        if (mode == Mode.RUN_TO_POSITION) {
            lift.setPower(liftPositionController.update(lift.getCurrentPosition()));
        } else if (mode == Mode.RESET) {
            if (limit.isPressed()) {
                lift.setPower(0);
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                mode = Mode.OPEN_LOOP;
            } else {
                lift.setPower(1);
            }
        }
        switch (wristPosition) {
            case IDLE:
                disarmGrabber();
                if (Math.abs(liftPositionController.getLastError()) < 5) setWristPosition(wristIdlePosition);
                if (clock.seconds() > (startTimestamp + wristDropDelay) && liftState != LiftState.LOWERED) {
                    mode = Mode.RESET;
                    liftState = LiftState.LOWERED;
                }
                break;
            case GRAB:
                setWristPosition(wristGrabPosition);
                if (clock.seconds() > (startTimestamp + wristDelay)) {
                    armGrabber();
                }
                break;
            case LIFT:
                //raiseLift();
                if (Math.abs(liftPositionController.getLastError()) < 20) {
                    setWristPosition(wristDeployPosition);
                } else {
                    setWristPosition(wristLiftPosition);
                }
                break;
            case DEPLOY:
                setWristPosition(wristDeployPosition);
                break;
        }
    }

    @Override
    public boolean isBusy() {
        return mode == Mode.OPEN_LOOP;
    }
}
