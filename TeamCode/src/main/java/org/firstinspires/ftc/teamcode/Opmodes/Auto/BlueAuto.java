package org.firstinspires.ftc.teamcode.Opmodes.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import kotlin.Unit;

@Autonomous
public class BlueAuto extends SkystoneAutoOpMode{
    public static final double STONE_WIDTH = 49;
    // left and right are switched bc i'm lazy
    public static double RIGHT_STONE_X = 63;
    public static double CENTER_STONE_X = 125;
    public static double LEFT_STONE_X = 184;

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
        robot.outtake.cycleWrist();
        robot.outtake.setWristPosition(0.23);
    }

    @Override
    protected void run(SkystoneAutoOpMode.SkystonePosition position) {
        //for some reason position estimation is bad so I manually set the position of the robot
        switch (position) {
            case LEFT:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .reverse()
                                .splineTo(new Pose2d(-44,0,Math.toRadians(45)))
                                .addMarker(() -> {robot.intake.toggleIntake(); return Unit.INSTANCE;})
                                .splineTo(new Pose2d(-64,-14,Math.toRadians(45)))
                                .build()
                );
                robot.drive.setPoseEstimate(new Pose2d(-64,-14,Math.toRadians(45)));
                break;
            case RIGHT:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .reverse()
                                .splineTo(new Pose2d(-44,18,Math.toRadians(45)))
                                .addMarker(() -> {robot.intake.toggleIntake(); return Unit.INSTANCE;})
                                .splineTo(new Pose2d(-64,6,Math.toRadians(45)))
                                .build()
                );
                robot.drive.setPoseEstimate(new Pose2d(-64,6,Math.toRadians(45)));
                break;
            case CENTER:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .reverse()
                                .splineTo(new Pose2d(-44,8,Math.toRadians(45)))
                                .addMarker(() -> {robot.intake.toggleIntake(); return Unit.INSTANCE;})
                                .splineTo(new Pose2d(-64,-6,Math.toRadians(45)))
                                .build()
                );
                robot.drive.setPoseEstimate(new Pose2d(-64,-6,Math.toRadians(45)));
                break;
        }
        sleep(500);
        robot.intake.setSpeed(0);
        robot.outtake.cycleWrist();
        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .splineTo(new Pose2d(-36,28,Math.toRadians(90)))
                        .strafeTo(new Vector2d(-40,68))
                        .splineTo(new Pose2d(-76,96,Math.toRadians(170)))
                        .build()
        );
        robot.drive.engageHooks();
        robot.drive.setPoseEstimate(new Pose2d(-76,96,Math.toRadians(170)));
        sleep(500);
        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(-36,60,Math.toRadians(80)))
                        .setReversed(false)
                        .splineTo(new Pose2d(-24,108,Math.toRadians(60)))
                        .build()
        );
        robot.drive.setPoseEstimate(new Pose2d(-36,108,Math.toRadians(110)));
        robot.drive.disengageHooks();
        robot.outtake.cycleWrist();
        robot.outtake.cycleWrist();
        sleep(1000);
        robot.outtake.cycleWrist();
        robot.outtake.liftPositionDown();
        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .reverse()
                        .forward(56)
                        .build()
        );
    }
}
