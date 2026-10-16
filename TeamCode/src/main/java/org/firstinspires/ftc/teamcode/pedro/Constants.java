package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("fleft");
        c.frontRightName.set("fright");
        c.backLeftName.set("bleft");
        c.backRightName.set("bright");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-3.0387403833584523);
        c.yPodOffset.set(-8.889720045675443);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2404753740189706);
                Controller secondaryTranslationalForward = Controller.proportional(0.0888492250796735);
                Controller primaryTranslationalLateral = Controller.proportional(0.28853169582959765);
                Controller secondaryTranslationalLateral = Controller.proportional(0.10660475190013205);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01600209249394082));
                c.brake.set(Controller.proportionalFeedforward(0.013601778619849698));

                c.headingFeedback.set(Controller.proportional(4.24581815704888));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.052723491853234536, 0.002252386774984038));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06627054070977445, 0.07250302505909677));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0010579959792725857, 9.122093195915429E-4));

                c.maxAchievableForwardVelocity.set(64.09462139194844);
                c.maxAchievableStrafeVelocity.set(51.801964418077866);
                c.naturalForwardDeceleration.set(38.66766739098275);
                c.naturalStrafeDeceleration.set(64.18417208088245);
            }
    );


    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}