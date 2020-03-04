package org.firstinspires.ftc.teamcode.Opmodes;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake2;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Util.StickyGamepad;
import org.openftc.revextensions2.ExpansionHubMotor;

@TeleOp
public class Teleop extends OpMode {
    Robot robot;

    StickyGamepad stickyGamepad1, stickyGamepad2;

    double t = -1;

    ExpansionHubMotor park;

    boolean foundationAlign;

    public static PIDCoefficients foundationDistanceCoefficients = new PIDCoefficients(-0.05,0,0);
    public static PIDCoefficients foundationAngleCoefficients = new PIDCoefficients(-1,0,0);
    public static double foundationAlignDistance = 8;
    public static final double TOF_SENSOR_DISTANCE = 10.3950008;
    private PIDFController foundationDistanceController;
    private PIDFController foundationAngleController;

    Rev2mDistanceSensor TOF1;
    Rev2mDistanceSensor TOF2;

    @Override
    public void init() {
        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);
        robot = new Robot(this);
        robot.start();
        robot.drive.setEncoderMode(true);

        telemetry.addLine("God is dead and we have killed him");
        telemetry.update();
        int soundID = hardwareMap.appContext.getResources().getIdentifier("toad_here_we_go","raw",hardwareMap.appContext.getPackageName());
        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundID);

        park = hardwareMap.get(ExpansionHubMotor.class, "park");

        TOF1 = hardwareMap.get(Rev2mDistanceSensor.class, "TOF1");
        TOF2 = hardwareMap.get(Rev2mDistanceSensor.class, "TOF2");
        foundationDistanceController = new PIDFController(foundationDistanceCoefficients);
        foundationAngleController = new PIDFController(foundationAngleCoefficients);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        stickyGamepad1.update();
        stickyGamepad2.update();
        if (stickyGamepad1.right_bumper) t = -t;
        if (stickyGamepad1.right_stick_button) {
            if (Math.abs(t) == 1) t *= 0.3;
            else  t *= 10.0/3;
        }

        double d1 = TOF1.getDistance(DistanceUnit.CM) * 0.6657317711 + 2.33512549;
        double d2 = TOF2.getDistance(DistanceUnit.CM) * 0.6343905382 + 2.814252421;

        double distance = (d1 + d2) / 2;
        double angle = Math.atan2(d1-d2, TOF_SENSOR_DISTANCE);
        if (stickyGamepad1.x) foundationAlign = !foundationAlign;

        if (gamepad1.right_stick_x != 0) robot.drive.setDrivePower(new Pose2d(0,0, -Math.abs(t) * gamepad1.right_stick_x));
        else if (foundationAlign) {
            double distanceCorrection = foundationDistanceController.update(distance);
            double angleCorrection = foundationAngleController.update(angle);
            robot.drive.setDrivePower(new Pose2d(distanceCorrection,t*gamepad1.left_stick_x,angleCorrection));
        }
        else robot.drive.setDrivePower(new Pose2d(t*gamepad1.left_stick_y,t*gamepad1.left_stick_x,0));
        robot.intake.setSpeed(0.8*gamepad1.left_trigger);
        if (gamepad1.left_bumper) {
            robot.intake.setSpeed(-1);
        }

        //telemetry.addData("intake power", robot.intake.getSpeed());
        if (gamepad1.a) robot.drive.engageHooks();
        if (gamepad1.b) robot.drive.disengageHooks();
        if (gamepad1.x) {
            robot.stackalign.setDo_align(robot.outtake.liftPosition);
        }

        if (robot.stackalign.do_align) {
            robot.drive.setDrivePower(new Pose2d(robot.stackalign.verticalCorrection, robot.stackalign.horizontalCorrection,
                   robot.stackalign.angelCorrection));
        }
        robot.outtake.setLiftPower(gamepad2.left_stick_y);
        if (stickyGamepad2.dpad_up) robot.outtake.liftPositionUp();
        else if (stickyGamepad2.dpad_down) robot.outtake.liftPositionDown();

        //if (gamepad2.a) robot.outtake.armGrabber();
        if (gamepad2.a) robot.outtake.backToIdle();
        if (gamepad2.b) robot.outtake.disarmGrabber();
        if (gamepad2.right_bumper) {
            if (gamepad2.y) {
                robot.outtake.setWristPosition(Outtake2.wristGrabPosition);
            } else if (gamepad2.x) {
                robot.outtake.setWristPosition(Outtake2.wristDeployPosition);
            } else {
                robot.outtake.setWristPosition(gamepad2.right_trigger);
            }
        } else if (stickyGamepad2.x) {
            robot.outtake.cycleWrist();
        }
        if (stickyGamepad2.y) robot.outtake.jumpStages();
        if (gamepad2.dpad_left) robot.outtake.cap();
        //telemetry.addData("switch mode engaged", t);
      //  telemetry.addData("wrist position", robot.outtake.getWristPosition());
        park.setPower(gamepad2.left_trigger);
    //    telemetry.addData("lift position", robot.outtake.getLiftPosition());

    //   telemetry.addData("lift target position", robot.outtake.getLiftTargetPosition());
       telemetry.addData("lift stages", robot.outtake.liftPosition);
      //  telemetry.addData("lift mode", robot.outtake.getMode());
       // telemetry.addData("power", robot.outtake.power);
      //  telemetry.addData("controlleroutput", robot.outtake.controlleroutput);
        telemetry.addData("d3", robot.stackalign.d3);
        telemetry.addData("d4", robot.stackalign.d4);
        telemetry.addData("d1", robot.stackalign.d1);
        telemetry.addData("d2", robot.stackalign.d2);
        telemetry.addData("horizontalCorrection", robot.stackalign.horizontalCorrection);
        telemetry.addData("verticalCorrection", robot.stackalign.verticalCorrection);
        telemetry.addData("angelCorrection", robot.stackalign.angelCorrection);

        if (gamepad2.left_bumper) park.setPower(-1);
    }
}
