package org.firstinspires.ftc.teamcode.Opmodes.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.SideGrippers;

import kotlin.Unit;

@Autonomous
public class BlueAuto extends SkystoneAutoOpMode{
    public static final double STONE_WIDTH = 49;
    // left and right are switched bc i'm lazy
    public static double RIGHT_STONE_X = 100;
    public static double CENTER_STONE_X = 160;
    public static double LEFT_STONE_X = 220;

    @Override
    protected SkystoneAutoOpMode.SkystonePosition getSkystonePosition() {
        double xPos = detector.getScreenPosition().x;
        if (Math.abs(RIGHT_STONE_X - xPos) < STONE_WIDTH/2) return SkystoneAutoOpMode.SkystonePosition.RIGHT;
        else if (Math.abs(CENTER_STONE_X - xPos) < STONE_WIDTH/2) return SkystoneAutoOpMode.SkystonePosition.CENTER;
        else if (Math.abs(LEFT_STONE_X - xPos) < STONE_WIDTH/2) return SkystoneAutoOpMode.SkystonePosition.LEFT;
        return SkystoneAutoOpMode.SkystonePosition.RIGHT;
    }

    @Override
    protected void setup() {
        robot.drive.disengageHooks();
        robot.drive.setPoseEstimate(new Pose2d());
        robot.sideGrippers.setGripperSide(SideGrippers.GripperSide.RIGHT);
    }

    @Override
    protected void run(SkystoneAutoOpMode.SkystonePosition position) {
        robot.sideGrippers.cycleGripper();
/*
        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(-24,-8,Math.toRadians(90)))
                        .build()
        );
*/
        switch (position) {
            case LEFT:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .reverse()
                                .splineTo(new Pose2d(-23,8,Math.toRadians(-90)))
                                .strafeTo(new Vector2d(-32,33))
                                .build()
                );
                break;
            case RIGHT:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .reverse()
                                .splineTo(new Pose2d(-23,8,Math.toRadians(-90)))
                                .strafeTo(new Vector2d(-31,14))
                                .build()
                );
                break;
            case CENTER:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .reverse()
                                .splineTo(new Pose2d(-23,8,Math.toRadians(-90)))
                                .strafeTo(new Vector2d(-32,22))
                                .build()
                );
                break;
        }

        robot.sideGrippers.cycleGripper();
        sleep(500);
        robot.sideGrippers.cycleGripper();

        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(-21,-40))
                        .strafeTo(new Vector2d(-36,-84))
                        .build()
        );

        robot.sideGrippers.cycleGripper();
        sleep(750);
        robot.sideGrippers.cycleGripper();
        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(-25,-36))
                        .reverse()
                        .splineTo(new Pose2d(-25,-16,Math.toRadians(-90)))
                        .build()
        );

        robot.sideGrippers.cycleGripper();

        switch (position) {
            case CENTER:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .reverse()
                                .strafeTo(new Vector2d(-35,1))
                                .build()
                );
                break;
            case LEFT:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .reverse()
                                .strafeTo(new Vector2d(-34,10))
                                .build()
                );
                break;
            case RIGHT:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .reverse()
                                .strafeTo(new Vector2d(-35,-7))
                                .build()
                );
                break;
        }
        robot.sideGrippers.cycleGripper();
        sleep(500);
        robot.sideGrippers.cycleGripper();

        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(-24,-40))
                        .strafeTo(new Vector2d(-39,-84))
                        .build()
        );

        robot.sideGrippers.cycleGripper();
        sleep(750);
        robot.sideGrippers.cycleGripper();

        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .strafeLeft(8)
                        .build()
        );
        robot.drive.turnSync(Math.toRadians(180));
        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .strafeLeft(8)
                        .forward(12)
                        .build()
        );
        robot.drive.engageHooks();
        sleep(500);
        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(-12,-56,Math.toRadians(-90)))
                        .build()
        );
        //this is what happens when you don't normalize the heading -_-
        robot.drive.turnSync(Math.toRadians(270));
        robot.drive.disengageHooks();
        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(-30,-29,Math.toRadians(-90)))
                        .build()
        );
    }
}
