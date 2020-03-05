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
    public int liftPosition;
    public static final double LIFT_ITERATION = 4.0;
    public static final double PULLEY_RADIUS = 2.0/2;
    public static final double TICKS_PER_REV = 537.6;
    public static final double kp = 0.8; // rev 0.6,
    public static final double ki = 0.001;
    public static final double kD = 0.0;
    private PIDFController liftPositionController;
    public static PIDCoefficients LiftPIDCoefficients = new PIDCoefficients(kp,ki, kD);
    public static double liftKG = 0.0; //0.01
    public static double kstatic = 0;
    public static double pulley_adjust = 0.96;
    public static double liftlockPower = 0.03;

    ExpansionHubServo wristLeft;
    ExpansionHubServo wristRight;
    public static double wristDeployPosition = 0.61; // 0.6
    public static double wristGrabPosition = 0.17;
    public static double wristIdlePosition = 0.25; //prev, 0.22
    public static double wristLiftPosition = 0.31;   // 0.22
    public static double wristDelay = 0.5;
    public static double liftUpTimeout = 1.0;
    public static double LiftDropDelay = 1.0;

    ExpansionHubServo grabber;
    public static double grabberArmPosition = 0.98;
    public static double grabberDisarmPosition = 0.6;
    public static double grabberCapPosition = 0.17;

    public static double adjustStep = 0.9; // 1 inch
    boolean keylock = false;

    RevTouchSensor limit;

    public enum Mode {
        OPEN_LOOP,
        RUN_TO_POSITION,
        RESET,
        IDLE
    }

    public enum WristPosition {
        GRAB,
        GRAB_DONE,
        PRE_LIFT,
        LIFT,
        LIFT_DONE,
        DEPLOY,
        DEPLOY_DONE,
        PRE_DROP,
        DROP,
        DROP_DONE,
        IDLE
    }

    public enum LiftState {
        RAISED,
        LOWERED
    }

    public enum GrabState {
        GRABSTONE,
        IDLE
    }
    private Mode mode;
    private WristPosition wristPosition;
    private LiftState liftState;
    private GrabState grabstate;
    public double power;
    public double controlleroutput;

    NanoClock clock;
    double startTimestamp;



    public Outtake2(HardwareMap hardwareMap) {
        lift = hardwareMap.get(ExpansionHubMotor.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        liftPositionController = new PIDFController(LiftPIDCoefficients,0,0, 0, (x) ->liftKG );
        liftPositionController.setOutputBounds(-300,300);

        wristLeft = hardwareMap.get(ExpansionHubServo.class, "wristLeft");
        wristRight = hardwareMap.get(ExpansionHubServo.class, "wristRight");
        wristLeft.setPwmRange(new PwmControl.PwmRange(500,2500));
        wristRight.setPwmRange(new PwmControl.PwmRange(500,2500));
        wristLeft.setDirection(Servo.Direction.REVERSE);

        grabber = hardwareMap.get(ExpansionHubServo.class, "grabber");

        limit = hardwareMap.get(RevTouchSensor.class, "limit");

        mode = Mode.IDLE;
        wristPosition = WristPosition.IDLE;
        liftState = LiftState.LOWERED;

        clock = NanoClock.system();
        liftPosition = 0;
        grabstate =  GrabState.IDLE;
        setWristPosition(wristIdlePosition);
        disarmGrabber();
        startTimestamp = clock.seconds();
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        double right = (position - 0.17) *.94 + 0.15;
        wristLeft.setPosition(position);
        wristRight.setPosition(right);
    }

    public WristPosition getWristPosition() {
        return wristPosition;
    }

    public void armGrabber() {
        grabber.setPosition(grabberArmPosition);
        grabstate = GrabState.GRABSTONE;
    }

    public void disarmGrabber() {
        grabber.setPosition(grabberDisarmPosition);
        grabstate= GrabState.IDLE;
    }

    public void setGrabberPosition (double position) {
        grabber.setPosition(position);
    }

    private void liftControllerSetPosition()
    {
        double position;

        liftPositionController.reset();
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (liftPosition <= 0) {
            position = 0;
            liftPosition = 0;
        } else if (liftPosition == 1) {
            position =  encoderInchesToTicks(1.6);
         } else {
            position = (liftPosition - 1) * encoderInchesToTicks(LIFT_ITERATION) + encoderInchesToTicks(1.8);
            position = position * pulley_adjust;
        }
        liftPositionController.setTargetPosition(position);
    }

    private void liftControllerAdjustPosition(double adjust){
        double position = lift.getCurrentPosition();
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftPositionController.reset();
        position =  position + encoderInchesToTicks(adjust);
        liftPositionController.setTargetPosition(position);
        //lift.setTargetPosition((int) position);
    }


    private double positionToPower( double position)
    {
        double k;

        if (Math.abs(position) > 300) {
            k = 1.0;
        } else {
            k = Math.abs(position/300);
        }
        power  = k * Math.signum(position);

        return power;
    }

    public void raiseLift(double step) {

        liftControllerAdjustPosition(step);
        //liftControllerSetPosition();
    }

    public void lowerLift() {
        mode = Mode.RUN_TO_POSITION;
        liftPositionController.setTargetPosition(0);
    }

    public void liftPositionUp() {
        mode = Mode.RUN_TO_POSITION;
       // if (liftPosition != 8) liftPosition ++;
        raiseLift(adjustStep);
    }

    public void liftPositionDown() {
        mode = Mode.RUN_TO_POSITION;
      //  if (liftPosition != 0) liftPosition --;
        raiseLift(- adjustStep);
    }

    public void cap() {
        if (wristPosition == WristPosition.DEPLOY_DONE)
        grabber.setPosition(grabberCapPosition);
    }

    public void cycleWrist() {
        if (keylock) {
            keylock = false;
        }
        switch (wristPosition) {
            case IDLE:
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftPositionController.setTargetPosition(0);
                startTimestamp = clock.seconds();
                wristPosition = WristPosition.GRAB;
                mode = Mode.RUN_TO_POSITION;
                //always disarm Grabber in this position
                disarmGrabber();
                setWristPosition(wristGrabPosition);
                break;
            case GRAB_DONE:
                liftPosition ++;
                if (liftPosition > 8) {
                    liftPosition = 1;
                }
                //moving to lift position
                startTimestamp = clock.seconds();
                wristPosition = WristPosition.PRE_LIFT;
                setWristPosition(wristLiftPosition);
                break;
            case LIFT_DONE:
                wristPosition = WristPosition.DEPLOY;
                setWristPosition(wristDeployPosition);
                startTimestamp = clock.seconds();

                break;
            case DEPLOY_DONE:
                //waiting for grab to release
                if (grabstate == GrabState.IDLE) {
                    liftControllerAdjustPosition(3);
                    //liftPositionController.reset();
                    //mode = Mode.OPEN_LOOP;
                    startTimestamp = clock.seconds();
                    setWristPosition(wristLiftPosition);
                    //setWristPosition(wristIdlePosition);
                    wristPosition = WristPosition.PRE_DROP;
                }
                break;
            case DROP_DONE:
                disarmGrabber();
                mode = Mode.IDLE;
                lift.setPower(0);
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftPositionController.setTargetPosition(0);
                setWristPosition(wristIdlePosition);
                wristPosition = WristPosition.IDLE;
                break;
                default:
                    break;
        }
    }

    public void backToIdle() {
        if (keylock)
            return;
        if (liftPosition >= 1) {
            liftPosition--;
        }
        switch (wristPosition) {

            case DEPLOY_DONE:
                liftControllerAdjustPosition(3);
            case GRAB_DONE:
            case LIFT_DONE:
                startTimestamp = clock.seconds();
                setWristPosition(wristLiftPosition);
                wristPosition = WristPosition.PRE_DROP;
                break;
            default:
                break;
        }
    }

    public void jumpStages() {
        if (liftPosition < 8) {
            liftPosition++;
        }
    }

    public double getLiftPosition() {
        return encoderTicksToInches(lift.getCurrentPosition());
    }

    public double getLiftTargetPosition() {
        return encoderTicksToInches(liftPositionController.getTargetPosition());
    }

    public static double encoderTicksToInches (double ticks) {
        return PULLEY_RADIUS * 2 * Math.PI * ticks / TICKS_PER_REV;
    }

    public static int encoderInchesToTicks (double inches) {
        return (int) (inches * TICKS_PER_REV / PULLEY_RADIUS / 2 / Math.PI);
    }

    @Override
    public void update() {
        if (this.mode == Mode.RUN_TO_POSITION) {
            controlleroutput = liftPositionController.update(lift.getCurrentPosition());

            power = positionToPower(liftPositionController.update(lift.getCurrentPosition()));
            lift.setPower(power);
        } else if (this.mode == Mode.RESET) {
            if (limit.isPressed()) {
                lift.setPower(0);
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                liftPositionController.setTargetPosition(0);
                 mode = Mode.RUN_TO_POSITION;
                wristPosition = WristPosition.IDLE;
                setWristPosition(wristIdlePosition);
                liftState = LiftState.LOWERED;
            } else {
              lift.setPower(-0.6);
            }
        }
        switch (wristPosition) {
            case IDLE:
                if (liftState != LiftState.LOWERED) {
                    lift.setPower(-0.2);
                }
                break;
            case GRAB:
                // setWristPosition(wristGrabPosition);
                if (clock.seconds() > (startTimestamp + wristDelay)) {
                    armGrabber();
                    wristPosition = WristPosition.GRAB;
                    //liftState = LiftState.RAISED;
                    wristPosition = WristPosition.GRAB_DONE;
                }
                break;
            case PRE_LIFT:
                if (clock.seconds() > (startTimestamp + wristDelay)) {
                    wristPosition = WristPosition.LIFT;

                    /* start lifting */
                    startTimestamp = clock.seconds();
                    liftControllerSetPosition();


                }
                break;
            case LIFT:
                if (Math.abs(liftPositionController.getLastError()) < 20 || clock.seconds() > (startTimestamp + liftUpTimeout)) {
                    wristPosition = WristPosition.LIFT_DONE;
                }

                break;
            case DEPLOY:
                if (clock.seconds() > (startTimestamp + wristDelay)) {
                    wristPosition = WristPosition.DEPLOY_DONE;
                }
                break;
            case PRE_DROP:
                if (clock.seconds() > (startTimestamp + 0.8)) {
                    setWristPosition(wristLiftPosition);
                    this.mode = Mode.RESET;
                    wristPosition = WristPosition.DROP;
                    startTimestamp = clock.seconds();
                   // lift.setPower(-1);
                    //liftPositionController.reset();
                    //lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    //liftPositionController.setTargetPosition(0);
                }
                break;
            case DROP:
              //  if (Math.abs(liftPositionController.getLastError()) < 20 || clock.seconds() > (startTimestamp + LiftDropDelay)) {
                //time out if mode.rest doesn't transition to idle
                if (clock.seconds() > (startTimestamp + LiftDropDelay)) {
                    disarmGrabber();
                    mode = Mode.IDLE;
                    lift.setPower(0);
                    lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    liftPositionController.setTargetPosition(0);
                    setWristPosition(wristIdlePosition);
                    wristPosition = WristPosition.IDLE;
                    mode = Mode.RUN_TO_POSITION;
                    liftState = LiftState.LOWERED;

                }
                break;

            default:
                break;
        }
    }

    @Override
    public boolean isBusy() {
        return mode == Mode.OPEN_LOOP;
    }
}
