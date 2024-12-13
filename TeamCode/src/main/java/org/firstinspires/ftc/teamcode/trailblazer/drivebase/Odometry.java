package org.firstinspires.ftc.teamcode.trailblazer.drivebase;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.fotmrobotics.trailblazer.MathKt;
import org.fotmrobotics.trailblazer.Pose2D;

public class Odometry {
    HardwareMap hardwareMap;

    private SparkFunOTOS OTOS;
    String deviceName = "otos";

    Pose2D currentPos;

    // TODO: Have options listed here
    //  include units, offsets, and scalars

    public Odometry(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        SparkFunOTOS sparkFunOTOS = hardwareMap.get(SparkFunOTOS.class, deviceName);
        OTOS = sparkFunOTOS;
        OTOS.setLinearUnit(SparkFunOTOS.LinearUnit.INCHES);
        OTOS.setAngularUnit(SparkFunOTOS.AngularUnit.DEGREES);
        OTOS.setLinearScalar(1.0d);
        OTOS.setAngularScalar(1.0d);
        OTOS.setOffset(new Pose2D(0.0d, -4.0d, 0.0d));
        OTOS.calibrateImu();
        OTOS.resetTracking();
        OTOS.setPosition(new Pose2D(0.0d, 0.0d, 0.0d));
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
}
