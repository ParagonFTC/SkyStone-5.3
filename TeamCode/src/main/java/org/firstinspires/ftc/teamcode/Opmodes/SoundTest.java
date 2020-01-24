package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Util.StickyGamepad;

@TeleOp(name = "Sound Test", group = "test")
public class SoundTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        int soundID = hardwareMap.appContext.getResources().getIdentifier("toad_here_we_go","raw",hardwareMap.appContext.getPackageName());
        StickyGamepad stickyGamepad1 = new StickyGamepad(gamepad1);
        waitForStart();
        while (!isStopRequested()) {
            if (stickyGamepad1.x) {
                telemetry.addAction(() -> SoundPlayer.getInstance().startPlaying(hardwareMap.appContext,soundID));
                telemetry.update();
            }
        }
    }
}
