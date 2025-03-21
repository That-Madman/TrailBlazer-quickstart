package org.firstinspires.ftc.teamcode.trailblazer.drivebase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.fotmrobotics.trailblazer.PIDF;
import org.fotmrobotics.trailblazer.Pose2D;

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
    int[] reverseMotors = {};

    // TODO: Tune the PIDF loops.
    PIDF positionPID = new PIDF(1, 0,0,0);
    PIDF headingPID = new PIDF(1, 0,0,0);

    // TODO: Change if necessary.
    // Name of the SparkFunOTOS in the configuration.
    String SparkFunOTOS = "otos";

    // TODO: Change if necessary.
    // Position of the SparkFunOTOS relative to the center.
    Pose2D offset = new Pose2D(0,0, 0);

    // Units
    DistanceUnit linearUnit = DistanceUnit.INCH;
    AngleUnit angularUnit = AngleUnit.DEGREES;

    // TODO: Tune the linear and angular scalar.

    // Sets the linear scale for the SparkFunOTOS.
    double linearScalar = 1;

    // Sets the angular scale for the SparkFunOTOS.
    double angularScalar = 1;

    // Scale for speed.
    double xScale = 1;
    double yScale = 1;
    double angularScale = 1;
}
