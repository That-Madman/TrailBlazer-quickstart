package org.firstinspires.ftc.teamcode.trailblazer.drivebase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.fotmrobotics.trailblazer.PIDF;
import org.fotmrobotics.trailblazer.Vector2D;

/**
 * Edit all components here.
 */
public class DriveValues {
    // TODO: Change if necessary.
    /*
    Name of the motors in the configuration

    0. Front Left
    1. Front Right
    2. Back Left
    3. Back Right
    */
    String[] motorNames = {
            "frontLeft",
            "frontRight",
            "backLeft",
            "backRight"
    };

    // TODO: Reverse motors if necessary.
    int[] reverseMotors = {
            1,
            2,
            3
    };

    GoBildaPinpointDriver.EncoderDirection[]  odoDir = {
            GoBildaPinpointDriver.EncoderDirection.REVERSED,
            GoBildaPinpointDriver.EncoderDirection.REVERSED
    };

    // TODO: Tune the PIDF loops.
    PIDF positionPID = new PIDF(1, 0,0,0);
    PIDF headingPID = new PIDF(1, 0,0,0);

    // TODO: Change if necessary.
    // Name of the SparkFunOTOS in the configuration.
    String SparkFunOTOS = "otos";

    // TODO: Change if necessary.
    // Position of the GoBildaPinpointDriver relative to the center.
    Vector2D offset = new Vector2D(0,0);
    DistanceUnit offsetUnit = DistanceUnit.MM;

    // Units
    public DistanceUnit linearUnit = DistanceUnit.INCH;
    public AngleUnit angularUnit = AngleUnit.DEGREES;

    //Type of odom pod used
    public GoBildaPinpointDriver.GoBildaOdometryPods podType = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    // Scale for speed.
    double xScale = 1;
    double yScale = 1;
    double angularScale = 1;
}
