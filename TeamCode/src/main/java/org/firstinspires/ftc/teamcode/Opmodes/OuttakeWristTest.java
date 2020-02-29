package org.firstinspires.ftc.teamcode.Opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Outtake2;

@TeleOp
@Config
public class OuttakeWristTest extends LinearOpMode {
    public static double wristPosition = 0.5;
    public static double grabberPosition = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        Outtake2 outtake = new Outtake2(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            outtake.setWristPosition(wristPosition);
            outtake.setGrabberPosition(grabberPosition);
        }
    }
}
