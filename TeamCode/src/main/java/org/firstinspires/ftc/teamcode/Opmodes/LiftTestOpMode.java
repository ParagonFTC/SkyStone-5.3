package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.openftc.revextensions2.ExpansionHubMotor;

@TeleOp
public class LiftTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ExpansionHubMotor lift = hardwareMap.get(ExpansionHubMotor.class,"lift");
        telemetry.addData("Target Position Tolerance", lift.getTargetPositionTolerance());
        telemetry.addData("Position PID", lift.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        telemetry.update();
        waitForStart();
    }
}
