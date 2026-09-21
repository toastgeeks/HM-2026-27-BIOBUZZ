package org.firstinspires.ftc.teamcode.subSystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class IntakeCode {

    public static DcMotor intake;

    private int MIN_TICKS = 0;
    private static int MAX_TICKS = 4000;

    private double NUDGE_POWER = 0.5;
    private double RUN_TO_POSITION_POWER = 1;

    private DcMotorEx liftMotor;
    public int targetLiftTicks = 0;

    // =========================
    // INTAKE ANTI-JAM SETTINGS
    // =========================

    // How long the encoder can remain stationary before we call it a jam
    private static final long JAM_TIME_MS = 300;

    // How long to reverse when a jam is detected
    private static final long REVERSE_TIME_MS = 1000;

    // Minimum power required before jam detection is active
    private static final double MIN_INTAKE_POWER = 0.15;

    // =========================
    // INTAKE VARIABLES
    // =========================

    private static volatile double requestedIntakePower = 0.0;

    private volatile boolean intakeMonitorRunning = false;
    private Thread intakeMonitorThread;

    // =========================
    // INIT
    // =========================

    public void init(HardwareMap hwMap) {

        intake = hwMap.get(DcMotor.class, "intake");

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // Start the intake anti-jam monitor
        startIntakeMonitor();

    }

    // =========================
    // INTAKE CONTROL
    // =========================

    public static void setIntakeSpeed(double intakeSpeed) {

        requestedIntakePower = Range.clip(intakeSpeed, -1.0, 1.0);

        // Normally set the requested power immediately.
        // The monitor thread will temporarily override this if a jam occurs.
        intake.setPower(requestedIntakePower);
    }

    // =========================
    // INTAKE ANTI-JAM MONITOR
    // =========================

    private void startIntakeMonitor() {

        intakeMonitorRunning = true;

        intakeMonitorThread = new Thread(() -> {

            int lastPosition = intake.getCurrentPosition();
            long lastMovementTime = System.currentTimeMillis();

            while (intakeMonitorRunning) {

                double power = requestedIntakePower;
                int currentPosition = intake.getCurrentPosition();

                // Only check for jams while the intake is actually supposed
                // to be running.
                if (Math.abs(power) >= MIN_INTAKE_POWER) {

                    // Encoder has moved
                    if (currentPosition != lastPosition) {

                        lastPosition = currentPosition;
                        lastMovementTime = System.currentTimeMillis();
                    }

                    // Encoder has NOT moved for long enough
                    else if (System.currentTimeMillis() - lastMovementTime >= JAM_TIME_MS) {

                        // =========================
                        // JAM DETECTED
                        // =========================

                        // Reverse the intake
                        intake.setPower(-power);

                        // Reverse for 1 second
                        try {
                            Thread.sleep(REVERSE_TIME_MS);
                        } catch (InterruptedException e) {
                            Thread.currentThread().interrupt();
                            break;
                        }

                        // Return to normal direction
                        intake.setPower(requestedIntakePower);

                        // Reset the movement timer
                        lastPosition = intake.getCurrentPosition();
                        lastMovementTime = System.currentTimeMillis();
                    }

                } else {

                    // Intake isn't running, so reset the jam timer
                    lastPosition = currentPosition;
                    lastMovementTime = System.currentTimeMillis();
                }

                // Check the encoder roughly every 50 ms
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        });

        // Allows the thread to end automatically when the program ends
        intakeMonitorThread.setDaemon(true);
        intakeMonitorThread.start();
    }

    // Stop the anti-jam monitor if needed
    public void stopIntakeMonitor() {

        intakeMonitorRunning = false;

        if (intakeMonitorThread != null) {
            intakeMonitorThread.interrupt();
        }

        if (intake != null) {
            intake.setPower(0);
        }

        requestedIntakePower = 0;
    }
}