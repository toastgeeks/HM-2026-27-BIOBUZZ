package org.firstinspires.ftc.teamcode.subSystems.Turret;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subSystems.Turret.ServoTester;

@Autonomous
public class ServoTesterOpMode extends OpMode {

    ServoTester servo = new ServoTester();

    private TelemetryManager panelsTelemetry;

    private final ElapsedTime timer = new ElapsedTime();

    private final TurretMechanism turret =
            new TurretMechanism();

    public void init() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        turret.init(hardwareMap);

        servo.init(hardwareMap);

        panelsTelemetry.addLine(
                "Turret Tuning Mode"
        );
        panelsTelemetry.update();
        timer.reset();


    }
    private boolean message_print[] = new boolean[6];
    public void start() {
    try {
        panelsTelemetry.addLine("Starting timer");
        while (timer.milliseconds() < 5000) {
            servo.setServoPower(0.05);
            if (!message_print[0]) {
                panelsTelemetry.addLine("Servo Power = 0.05");
                panelsTelemetry.update();
                message_print[0] = true;
            }
        }
        while (timer.milliseconds() < 10000) {
            servo.setServoPower(-0.1);
            if (!message_print[1]) {
                panelsTelemetry.addLine("Servo Power = 0.1");
                message_print[1] = true;
            }

        }
        while (timer.milliseconds() < 15000) {
            servo.setServoPower(0.15);
            if (!message_print[2]) {
                panelsTelemetry.addLine("Servo Power = 0.15");
                message_print[2] = true;
            }
        }
        while (timer.milliseconds() < 18000) {
            servo.setServoPower(-0.2);
            if (!message_print[3]) {
                panelsTelemetry.addLine("Servo Power = 0.2");
                message_print[3] = true;
            }

        }
        while (timer.milliseconds() < 21000) {
            servo.setServoPower(0.25);
            if (!message_print[4]) {
                panelsTelemetry.addLine("Servo Power = 0.25");
                message_print[4] = true;
            }
        }

        servo.setServoPower(0.0);
    } catch (Exception e) {
        String error = e.getMessage();
        panelsTelemetry.debug("ERROR: ", error);
    }
    }

    public void loop(){
        panelsTelemetry.update();
    }

    public void stop() {
        servo.setServoPower(0.0);
    }

}
