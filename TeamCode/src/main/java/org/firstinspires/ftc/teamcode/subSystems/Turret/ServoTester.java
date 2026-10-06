package org.firstinspires.ftc.teamcode.subSystems.Turret;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ServoTester{
    private CRServo turret;
    private TelemetryManager panelsTelemetry;


    public void init(HardwareMap hwMap){
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        turret = hwMap.get(
                CRServo.class,
                "turret"
        );


        turret.setPower(0);
    }

    public void setServoPower(double speed) {
        turret.setPower(speed);
    }

    public void stop() {
        setServoPower(0.0);
    }

}
