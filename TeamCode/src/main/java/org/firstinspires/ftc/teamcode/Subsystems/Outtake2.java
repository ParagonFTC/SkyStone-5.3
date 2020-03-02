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
    public static final double kp = 0.6; // rev 0.006,
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
    public static double wristDeployPosition = 0.6;
    public static double wristGrabPosition = 0.17;
    public static double wristIdlePosition = 0.22;
    public static double wristLiftPosition = 0.40;   // 0.22
    public static double wristDelay = 0.5;
    public static double liftUpTimeout = 1.0 ;
    public static double wristDropDelay = 1;

    ExpansionHubServo grabber;
    public static double grabberArmPosition = 0.98;
    public static double grabberDisarmPosition = 0.6;
    public static double grabberCapPosition = 0.17;

    RevTouchSensor limit;

    public enum Mode {
        OPEN_LOOP,
        RUN_TO_POSITION,
        RESET,
        IDLE
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
        wristLeft.setPwmRange(new PwmControl.PwmRange(500,1500));
        wristRight.setPwmRange(new PwmControl.PwmRange(500,1500));
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
        wristLeft.setPosition(position);
        wristRight.setPosition(position);
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
        position = (liftPosition - 1) * encoderInchesToTicks(LIFT_ITERATION) + encoderInchesToTicks(1);
        position = position * pulley_adjust;
        //lift.setTargetPosition((int) position);
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

    public void raiseLift() {
        // mode = Mode.RUN_TO_POSITION;
        //if (liftState == LiftState.RAISED)liftPositionController.setTargetPosition(-(liftPosition - 1) * encoderInchesToTicks(LIFT_ITERATION) - encoderInchesToTicks(1));
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftControllerSetPosition();
    }

    public void lowerLift() {
        mode = Mode.RUN_TO_POSITION;
        liftPositionController.setTargetPosition(0);
    }

    public void liftPositionUp() {
        if (liftPosition != 8) liftPosition ++;
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
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftPositionController.setTargetPosition(0);
                startTimestamp = clock.seconds();
                wristPosition = WristPosition.GRAB;
                mode = Mode.RUN_TO_POSITION;
                //always disarm Grabber in this position
                disarmGrabber();
                setWristPosition(wristGrabPosition);
                break;
            case GRAB:
                liftPosition ++;
                //raise first,  so it will not touch bottom
                startTimestamp = clock.seconds();
               // liftControllerAdjustPosition(1);
                setWristPosition(wristLiftPosition);
                wristPosition = WristPosition.LIFT;
                break;
            case LIFT:
                wristPosition = WristPosition.DEPLOY;
                startTimestamp = clock.seconds();
                liftControllerSetPosition();
                {
                    //setWristPosition(wristDeployPosition);
                    wristPosition = WristPosition.DEPLOY;
                }

                break;
            case DEPLOY:
               // liftControllerAdjustPosition(1);
                if (grabstate == GrabState.IDLE) {
                    liftPositionController.reset();
                    //mode = Mode.OPEN_LOOP;
                    startTimestamp = clock.seconds();
                    //setWristPosition(wristLiftPosition);
                    setWristPosition(wristIdlePosition);
                    wristPosition = WristPosition.IDLE;
                }
                break;
        }
    }

    public void deploy() {
        wristPosition = WristPosition.DEPLOY;
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
            // if (limit.isPressed()) {
                lift.setPower(0);
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                liftPositionController.setTargetPosition(0);
                 mode = Mode.RUN_TO_POSITION;
            //} else {
            //lift.setPower(1);
            //}
        }
        switch (wristPosition) {
            case IDLE:
                if (clock.seconds() > (startTimestamp + wristDelay)) {
                   // mode = Mode.RESET;
                    if (liftState != LiftState.LOWERED) {
                        this.mode = Mode.IDLE;
                        lift.setPower(-0.2);
                        liftPositionController.setTargetPosition(0);
                        liftState = LiftState.LOWERED;
                        disarmGrabber();
                        startTimestamp = clock.seconds();
                        setWristPosition(wristIdlePosition);
                    } else {
                        //lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                      //  liftPositionController.setTargetPosition(0);

                    }

                }
                if (liftState != LiftState.LOWERED) {
                    lift.setPower(-0.2);
                    liftPositionController.setTargetPosition(0);
                }
                break;
            case GRAB:
                // setWristPosition(wristGrabPosition);
                if (clock.seconds() > (startTimestamp + wristDelay)) {
                    armGrabber();
                    liftState = LiftState.RAISED;
                    if (liftPosition >  1) {
                        liftControllerSetPosition();
                    }
                }
                break;
            case LIFT:
                /*
                if (Math.abs(liftPositionController.getLastError()) < 20 || clock.seconds() > (startTimestamp + wristDelay)) {
                    setWristPosition(wristDeployPosition);
                    wristPosition = WristPosition.DEPLOY;

                } */
                //raiseLift();

                break;
            case DEPLOY:
                if (Math.abs(liftPositionController.getLastError()) < 20 || clock.seconds() > (startTimestamp + wristDelay)) {
                    setWristPosition(wristDeployPosition);
                }
                break;
        }
    }

    @Override
    public boolean isBusy() {
        return mode == Mode.OPEN_LOOP;
    }
}
