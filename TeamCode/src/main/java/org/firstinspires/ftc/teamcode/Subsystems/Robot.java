package org.firstinspires.ftc.teamcode.Subsystems;

import android.app.Activity;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.util.GlobalWarningSource;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;

public class Robot implements OpModeManagerNotifier.Notifications, GlobalWarningSource {
    public static final String TAG = "RobotLog";

    public MecanumDriveWrapper drive;
    public Intake intake;
    public Outtake2 outtake;
    public StackingAlign stackalign;
    public SideGrippers sideGrippers;
    private List<Subsystem> subsystems;
    private List<Subsystem> subsystemsWithProblems;

    private OpModeManagerImpl opModeManager;
    private ExecutorService subsystemUpdateExecutor;

    private boolean started;

    private Runnable subsystemUpdateRunnable = () -> {
        while (!Thread.currentThread().isInterrupted()) {
            try {
                for (Subsystem subsystem : subsystems) {
                    if (subsystem == null) continue;
                    try {
                        subsystem.update();
                        synchronized (subsystemsWithProblems) {
                            if (subsystemsWithProblems.contains(subsystem)) {
                                subsystemsWithProblems.remove(subsystem);
                            }
                        }
                    } catch (Throwable t) {
                        Log.w(TAG, "Subsystem update failed for " + subsystem.getClass().getSimpleName() + ": " + t.getMessage());
                        Log.w(TAG, t);
                        synchronized (subsystemsWithProblems) {
                            if (!subsystemsWithProblems.contains(subsystem)) {
                                subsystemsWithProblems.add(subsystem);
                            }
                        }
                    }
                }
            } catch (Throwable t) {
                Log.wtf(TAG, t);
            }
        }
    };

    public Robot(OpMode opMode) {
        subsystems = new ArrayList<>();

        try {
            drive = new MecanumDriveWrapper(opMode.hardwareMap);
        } catch (IllegalArgumentException e) {
            Log.w(TAG, "skipping Mecanum Drive");
        }

        try {
            intake = new Intake(opMode.hardwareMap);
        } catch (IllegalArgumentException e) {
            Log.w(TAG, "skipping Intake");
        }

        try {
            outtake = new Outtake2(opMode.hardwareMap);
        } catch (IllegalArgumentException e) {
            Log.w(TAG, "skipping Outtake");
        }

        try {
            sideGrippers = new SideGrippers(opMode.hardwareMap);
        } catch (IllegalArgumentException e) {
            Log.w(TAG, "skipping side grippers");
        }
        try {
            stackalign = new StackingAlign(opMode.hardwareMap);
        } catch (IllegalArgumentException e) {
            Log.w(TAG, "skipping stacking align ");
        }


        Activity activity = (Activity) opMode.hardwareMap.appContext;
        opModeManager = OpModeManagerImpl.getOpModeManagerOfActivity(activity);
        if (opModeManager != null) {
            opModeManager.registerListener(this);
        }

        subsystemUpdateExecutor = ThreadPool.newSingleThreadExecutor("subsystem update");

        subsystemsWithProblems = new ArrayList<>();
        RobotLog.registerGlobalWarningSource(this);
        subsystems.add(intake);
        subsystems.add(outtake);
        subsystems.add(sideGrippers);
        subsystems.add(stackalign);
    }

    public void start() {
        if (!started) {
            subsystemUpdateExecutor.submit(subsystemUpdateRunnable);
        }
    }

    private void stop() {
        if (subsystemUpdateExecutor != null) {
            subsystemUpdateExecutor.shutdownNow();
            subsystemUpdateExecutor = null;
        }

        RobotLog.unregisterGlobalWarningSource(this);
    }
    @Override
    public void onOpModePreInit(OpMode opMode) {

    }

    @Override
    public void onOpModePreStart(OpMode opMode) {

    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        stop();

        if (opModeManager != null) {
            opModeManager.unregisterListener(this);
            opModeManager = null;
        }
    }

    @Override
    public String getGlobalWarning() {
        List<String> globalWarnings = new ArrayList<>();
        synchronized (subsystemsWithProblems) {
            for (Subsystem subsystem : subsystemsWithProblems) {
                globalWarnings.add("Problem with" + subsystem.getClass().getSimpleName());
            }
        }
        return RobotLog.combineGlobalWarnings(globalWarnings);
    }

    @Override
    public void suppressGlobalWarning(boolean suppress) {

    }

    @Override
    public void setGlobalWarning(String warning) {

    }

    @Override
    public void clearGlobalWarning() {
        synchronized (subsystemsWithProblems) {
            subsystemsWithProblems.clear();
        }
    }

    public void sleep(double seconds) {
        try {
            Thread.sleep(Math.round(1000 * seconds));
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
