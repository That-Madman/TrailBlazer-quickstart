package org.firstinspires.ftc.teamcode.trailblazer.drivebase;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.fotmrobotics.trailblazer.MathKt;
import org.fotmrobotics.trailblazer.Pose2D;

public class Odometry {
    HardwareMap hardwareMap;

    DriveValues driveValues = new DriveValues();

    private SparkFunOTOS OTOS;

    Pose2D currentPos;

    public Odometry(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        OTOS = hardwareMap.get(SparkFunOTOS.class, driveValues.SparkFunOTOS);

        OTOS.setLinearUnit(driveValues.linearUnit);

        OTOS.setLinearScalar(driveValues.linearScalar);

        OTOS.setAngularUnit(driveValues.angularUnit);

        OTOS.setAngularScalar(driveValues.angularScalar);

        Pose2D offset = new Pose2D(2.5,-6.1875,0);
        OTOS.setOffset(driveValues.offset);

        OTOS.calibrateImu();
        OTOS.resetTracking();
        OTOS.setPosition(new Pose2D(0, 0, 0));
    }

    public void update() {
        Pose2D pos = OTOS.getPosition();
        pos.setH(MathKt.angleWrap(pos.getH()));
        currentPos = pos;
    }

    public Pose2D getPosition() {
        update();
        return currentPos;
    }

    public void resetHeading() {
        OTOS.calibrateImu();
    }

    public void resetPosition() {
        OTOS.setPosition(new Pose2D(0, 0, 0));
    }
}
