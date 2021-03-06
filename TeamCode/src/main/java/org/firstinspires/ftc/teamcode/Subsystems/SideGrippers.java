package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubServo;

@Config
public class SideGrippers implements Subsystem {
    private ExpansionHubServo leftWrist, rightWrist, leftGripper, rightGripper;

    public static double leftWristRetractPosition = 0.85;
    public static double rightWristRetractPosition = 0.3;
    public static double leftWristHoldPosition = 0.5;
    public static double rightWristHoldPosition = 0.6;
    public static double leftWristDeployPosition = 0.15;
    public static double rightWristDeployPosition = 0.99;

    public static double leftGripperRetractPosition = 0.6;
    public static double rightGripperRetractPosition = 0.4;
    public static double leftGripperGripPosition = 0.45;
    public static double rightGripperGripPosition = 0.6;
    public static double leftGripperReleasePosition = 0;
    public static double rightGripperReleasePosition = 1;

    public static double wristDelay = 0.5;
    private double wristStartTimestamp;

    private NanoClock clock;

    GripperPosition position;
    GripperSide side;

    public enum GripperSide {
        LEFT,
        RIGHT
    }

    public enum GripperPosition {
        RETRACT,
        HOLD,
        GRIP,
        RELEASE,
        RELEASE_2,
        HOLD_2
    }

    public SideGrippers(HardwareMap hardwareMap) {
        leftWrist = hardwareMap.get(ExpansionHubServo.class, "leftSideWrist");
        rightWrist = hardwareMap.get(ExpansionHubServo.class, "rightSideWrist");
        leftGripper = hardwareMap.get(ExpansionHubServo.class, "leftSideGripper");
        rightGripper = hardwareMap.get(ExpansionHubServo.class, "rightSideGripper");

        retract();
        side = GripperSide.RIGHT;

        clock = NanoClock.system();
    }

    public void setGripperSide(GripperSide side) {
        this.side = side;
    }

    public void retract() {
        leftWrist.setPosition(leftWristRetractPosition);
        rightWrist.setPosition(rightWristRetractPosition);
        leftGripper.setPosition(leftGripperRetractPosition);
        rightGripper.setPosition(rightGripperRetractPosition);
        position = GripperPosition.RETRACT;
    }

    public GripperSide getSide() {
        return side;
    }

    public GripperPosition getPosition() {
        return position;
    }

    public void cycleGripper() {
        switch (position) {
            case RETRACT:
            case HOLD_2:
                position = GripperPosition.RELEASE;
                switch (side) {
                    case RIGHT:
                        rightWrist.setPosition(rightWristDeployPosition);
                        rightGripper.setPosition(rightGripperReleasePosition);
                        break;
                    case LEFT:
                        leftWrist.setPosition(leftWristDeployPosition);
                        leftGripper.setPosition(leftGripperReleasePosition);
                        break;
                }
                break;
            case RELEASE:
                position = GripperPosition.GRIP;
                switch (side) {
                    case RIGHT:
                        rightWrist.setPosition(rightWristDeployPosition);
                        rightGripper.setPosition(rightGripperGripPosition);
                        break;
                    case LEFT:
                        leftWrist.setPosition(leftWristDeployPosition);
                        leftGripper.setPosition(leftGripperGripPosition);
                        break;
                }
                break;
            case GRIP:
                position = GripperPosition.HOLD;
                switch (side) {
                    case RIGHT:
                        rightWrist.setPosition(rightWristHoldPosition);
                        rightGripper.setPosition(rightGripperGripPosition);
                        break;
                    case LEFT:
                        leftWrist.setPosition(leftWristHoldPosition);
                        leftGripper.setPosition(leftGripperGripPosition);
                        break;
                }
                break;
            case HOLD:
                wristStartTimestamp = clock.seconds();
                position = GripperPosition.RELEASE_2;
                switch (side) {
                    case RIGHT:
                        rightWrist.setPosition(rightWristDeployPosition);
                        break;
                    case LEFT:
                        leftWrist.setPosition(leftWristDeployPosition);
                        break;
                }
                break;
            case RELEASE_2:
                wristStartTimestamp = clock.seconds();
                position = GripperPosition.HOLD_2;
                switch (side) {
                    case RIGHT:
                        rightWrist.setPosition(rightWristHoldPosition);
                        break;
                    case LEFT:
                        leftWrist.setPosition(leftWristHoldPosition);
                        break;
                }
                break;
        }
    }

    @Override
    public void update() {
        switch (side) {
            case RIGHT:
                switch (position) {
                    case RELEASE_2:
                        if (clock.seconds() > wristStartTimestamp + wristDelay) rightGripper.setPosition(rightGripperReleasePosition);
                        break;
                    case HOLD_2:
                        if (clock.seconds() > wristStartTimestamp + wristDelay) rightGripper.setPosition(rightGripperGripPosition);
                        break;
                }
                break;
            case LEFT:
                switch (position) {
                    case RELEASE_2:
                        if (clock.seconds() > wristStartTimestamp + wristDelay) leftGripper.setPosition(leftGripperReleasePosition);
                        break;
                    case HOLD_2:
                        if (clock.seconds() > wristStartTimestamp + wristDelay) leftGripper.setPosition(leftGripperGripPosition);
                        break;
                }
                break;
        }
    }

    @Override
    public boolean isBusy() {
        return false;
    }
}
