package org.firstinspires.ftc.teamcode.trailblazer.drivebase;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.fotmrobotics.trailblazer.MathKt;
import org.fotmrobotics.trailblazer.Pose2D;

/**
 * Controls the odometry. Currently supports the SparkFunOTOS.
 * To change the odometry method, change the methods corresponding to what they do.
 *
 * @author Preston Cokis
 */
public class Odometry {
    HardwareMap hardwareMap;

    DriveValues driveValues = new DriveValues();

    private SparkFunOTOS OTOS;

    Pose2D currentPos;
    Pose2D lastPos;

    public Odometry(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        OTOS = hardwareMap.get(SparkFunOTOS.class, driveValues.SparkFunOTOS);

        OTOS.setLinearUnit(driveValues.linearUnit);

        OTOS.setLinearScalar(driveValues.linearScalar);

        OTOS.setAngularUnit(driveValues.angularUnit);

        OTOS.setAngularScalar(driveValues.angularScalar);

        OTOS.setOffset(driveValues.offset);

        OTOS.calibrateImu();
        OTOS.resetTracking();
        OTOS.setPosition(new Pose2D(0, 0, 0));
    }

    /**
     * Updates the position values.
     */
    public void update() {
        lastPos = currentPos;

        Pose2D pos = OTOS.getPosition();
        pos.setH(MathKt.angleWrap(pos.getH()));

        currentPos = pos;
    }

    public Pose2D getPosition() {
        update();
        return currentPos;
    }

    public Pose2D getLastPosition() {
        return lastPos;
    }

    /**
     * Resets the heading.
     */
    public void resetHeading() {
        OTOS.calibrateImu();
    }

    /**
     * Resets the position.
     */
    public void resetPosition() {
        OTOS.setPosition(new Pose2D(0, 0, 0));
    }
}
