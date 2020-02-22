package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubServo;

@Config
public class SideGrippers implements Subsystem {
    private ExpansionHubServo leftWrist, rightWrist, leftGripper, rightGripper;

    private double leftWristPosition, rightWristPosition, leftGripperPosition, rightGripperPosition;

    public static double leftWristRetractPosition = 0.85;
    public static double rightWristRetractPosition = 0.3;
    public static double leftWristHoldPosition = 0.5;
    public static double rightWristHoldPosition = 0.65;
    public static double leftWristDeployPosition = 0.15;
    public static double rightWristDeployPosition = 0.99;

    public static double leftGripperRetractPosition = 0.6;
    public static double rightGripperRetractPosition = 0.4;
    public static double leftGripperGripPosition = 0.4;
    public static double rightGripperGripPosition = 0.6;
    public static double leftGripperReleasePosition = 0;
    public static double rightGripperReleasePosition = 1;

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
        RELEASE
    }

    public SideGrippers(HardwareMap hardwareMap) {
        leftWrist = hardwareMap.get(ExpansionHubServo.class, "leftSideWrist");
        rightWrist = hardwareMap.get(ExpansionHubServo.class, "rightSideWrist");
        leftGripper = hardwareMap.get(ExpansionHubServo.class, "leftSideGripper");
        rightGripper = hardwareMap.get(ExpansionHubServo.class, "rightSideGripper");

        retract();
        side = GripperSide.RIGHT;
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
            case HOLD:
                leftWristPosition = leftWristDeployPosition;
                rightWristPosition = rightWristDeployPosition;
                leftGripperPosition = leftGripperReleasePosition;
                rightGripperPosition = rightGripperReleasePosition;
                position = GripperPosition.RELEASE;
                break;
            case RELEASE:
                leftWristPosition = leftWristDeployPosition;
                rightWristPosition = rightWristDeployPosition;
                leftGripperPosition = leftGripperGripPosition;
                rightGripperPosition = rightGripperGripPosition;
                position = GripperPosition.GRIP;
                break;
            case GRIP:
                leftWristPosition = leftWristHoldPosition;
                rightWristPosition = rightWristHoldPosition;
                leftGripperPosition = leftGripperGripPosition;
                rightGripperPosition = rightGripperGripPosition;
                position = GripperPosition.HOLD;
                break;
        }
    }

    @Override
    public void update() {
        switch (side) {
            case RIGHT:
                rightWrist.setPosition(rightWristPosition);
                rightGripper.setPosition(rightGripperPosition);
                break;
            case LEFT:
                leftWrist.setPosition(leftWristPosition);
                leftGripper.setPosition(leftGripperPosition);
                break;
        }
    }

    @Override
    public boolean isBusy() {
        return false;
    }
}
