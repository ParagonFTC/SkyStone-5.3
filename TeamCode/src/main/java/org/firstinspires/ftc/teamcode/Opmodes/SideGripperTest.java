package org.firstinspires.ftc.teamcode.Opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.SideGrippers;
import org.firstinspires.ftc.teamcode.Util.StickyGamepad;
import org.openftc.revextensions2.ExpansionHubServo;

@TeleOp
@Config
public class SideGripperTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        StickyGamepad stickyGamepad1 = new StickyGamepad(gamepad1);
        Robot robot = new Robot(this);
        waitForStart();
        while (opModeIsActive()) {
            stickyGamepad1.update();
            if (stickyGamepad1.x) {
                robot.sideGrippers.cycleGripper();
            }
            if (gamepad1.dpad_left) {
                robot.sideGrippers.setGripperSide(SideGrippers.GripperSide.LEFT);
            }
            if (gamepad1.dpad_right) {
                robot.sideGrippers.setGripperSide(SideGrippers.GripperSide.RIGHT);
            }
            telemetry.addData("side", robot.sideGrippers.getSide());
            telemetry.addData("position", robot.sideGrippers.getPosition());
            telemetry.update();
        }
    }
}
