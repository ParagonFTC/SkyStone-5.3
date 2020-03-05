package org.firstinspires.ftc.teamcode.Opmodes.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.SideGrippers;

@Config
@Autonomous
public class RedAuto extends SkystoneAutoOpMode {
    public static final double STONE_WIDTH = 49;
    public static double RIGHT_STONE_X = 187;
    public static double CENTER_STONE_X = 127;
    public static double LEFT_STONE_X = 69;

    @Override
    protected SkystonePosition getSkystonePosition() {
        double xPos = detector.getScreenPosition().x;
        if (Math.abs(RIGHT_STONE_X - xPos) < STONE_WIDTH/2) return SkystonePosition.RIGHT;
        else if (Math.abs(CENTER_STONE_X - xPos) < STONE_WIDTH/2) return SkystonePosition.CENTER;
        else if (Math.abs(LEFT_STONE_X - xPos) < STONE_WIDTH/2) return SkystonePosition.LEFT;
        return SkystonePosition.RIGHT;
    }

    @Override
    protected void setup() {
        robot.drive.disengageHooks();
        robot.drive.setPoseEstimate(new Pose2d());
        robot.sideGrippers.setGripperSide(SideGrippers.GripperSide.LEFT);
        //robot.outtake.setWristPosition(0.2);
    }

    @Override
    protected void run(SkystonePosition position) {
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
                                .splineTo(new Pose2d(-24,-8,Math.toRadians(90)))
                                .strafeTo(new Vector2d(-32,-32))
                                .build()
                );
                break;
            case RIGHT:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .reverse()
                                .splineTo(new Pose2d(-24,-8,Math.toRadians(90)))
                                .strafeTo(new Vector2d(-31,-14))
                                .build()
                );
                break;
            case CENTER:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .reverse()
                                .splineTo(new Pose2d(-24,-8,Math.toRadians(90)))
                                .strafeTo(new Vector2d(-32,-22))
                                .build()
                );
                break;
        }

        robot.sideGrippers.cycleGripper();
        sleep(500);
        robot.sideGrippers.cycleGripper();

        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(-22,40))
                        .strafeTo(new Vector2d(-37,84))
                        .build()
        );

        robot.sideGrippers.cycleGripper();
        sleep(750);
        robot.sideGrippers.cycleGripper();
        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(-26,36))
                        .reverse()
                        .splineTo(new Pose2d(-26,16,Math.toRadians(90)))
                        .build()
        );

        robot.sideGrippers.cycleGripper();

        switch (position) {
            case CENTER:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .reverse()
                                .strafeTo(new Vector2d(-38,-1))
                                .build()
                );
                break;
            case LEFT:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .reverse()
                                .strafeTo(new Vector2d(-38,-9))
                                .build()
                );
                break;
            case RIGHT:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .reverse()
                                .strafeTo(new Vector2d(-37,7))
                                .build()
                );
                break;
        }
        robot.sideGrippers.cycleGripper();
        sleep(500);
        robot.sideGrippers.cycleGripper();

        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(-24,40))
                        .strafeTo(new Vector2d(-40,84))
                        .build()
        );

        robot.sideGrippers.cycleGripper();
        sleep(750);
        robot.sideGrippers.cycleGripper();

        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .strafeRight(8)
                        .build()
        );
        robot.drive.turnSync(Math.toRadians(180));
        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .strafeRight(8)
                        .forward(12)
                        .build()
        );
        robot.drive.engageHooks();
        sleep(500);
        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(-12,56,Math.toRadians(90)))
                        .build()
        );
        robot.drive.turnSync(Math.toRadians(90));
        robot.drive.disengageHooks();
        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(-30,30,Math.toRadians(90)))
                        .build()
        );
    }
}
