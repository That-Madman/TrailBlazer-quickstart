package org.firstinspires.ftc.teamcode.trailblazer.drivebase;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.fotmrobotics.trailblazer.PIDF;
import org.fotmrobotics.trailblazer.Pose2D;
import org.fotmrobotics.trailblazer.Vector2D;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Preston Cokis
 */
public class MecanumDrive {
    HardwareMap hardwareMap;

    private List<DcMotor> motors = new ArrayList();
    String[] motorNames = {
            "frontLeft",
            "frontRight",
            "backLeft",
            "backRight"
    };

    private SparkFunOTOS OTOS;

    private PIDF positionPID = new PIDF(0.125, 0, 0, 0);
    private PIDF headingPID = new PIDF(0.05, 0, 0, 0);
    public boolean disablePID = false;

    public Pose2D currentPos = new Pose2D(Double.NaN, Double.NaN, Double.NaN);
    public Pose2D targetPos = new Pose2D(Double.NaN, Double.NaN, Double.NaN);

    public MecanumDrive(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        setMotors(this.motorNames);
        this.motors.get(0).setDirection(DcMotorSimple.Direction.REVERSE);
        this.motors.get(2).setDirection(DcMotorSimple.Direction.REVERSE);
        setSparkFunOTOS("otos");
    }

    public void setSparkFunOTOS(String deviceName) {
        SparkFunOTOS sparkFunOTOS = (SparkFunOTOS) this.hardwareMap.get(SparkFunOTOS.class, deviceName);
        this.OTOS = sparkFunOTOS;
        sparkFunOTOS.setLinearUnit(SparkFunOTOS.LinearUnit.INCHES);
        this.OTOS.setAngularUnit(SparkFunOTOS.AngularUnit.DEGREES);
        this.OTOS.setLinearScalar(1.0d);
        this.OTOS.setAngularScalar(1.0d);
        this.OTOS.setOffset(new Pose2D(0.0d, -4.0d, 0.0d));
        this.OTOS.calibrateImu();
        this.OTOS.resetTracking();
        this.OTOS.setPosition(new Pose2D(0.0d, 0.0d, 0.0d));
    }

    public void updatePosition () {
        this.currentPos.set(this.OTOS.getPosition());
    }

    public Pose2D getPosition () {
        updatePosition();
        return this.currentPos;
    }

    public void setMotors (String[] motorNames) {
        for (int i = 0; i <= 3; i++) {
            this.motors.add(this.hardwareMap.get(DcMotor.class, motorNames[i]));
        }
    }

    public void runMotors (double[] powers) {
        for (int i = 0; i <= 3; i++) {
            this.motors.get(i).setPower(powers[i]);
        }
    }

    public double[] powersToTarget () {
        updatePosition();

        Vector2D difference = targetPos.minus(currentPos);

        double angle = Math.toDegrees(Math.atan2(difference.getY(), difference.getX())) + 90;
        angle = angle <= 180 ? angle : angle - 360;
        angle = angle - currentPos.getH();

        double headingError = targetPos.getH() - currentPos.getH();
        headingError = headingError > 180 ? headingError - 360 : headingError < -180 ? headingError - 360 : headingError;

        double out;
        if (!disablePID) {
            out = positionPID.update(difference.norm());
        } else {
            out = 1;
        }
        double headingOut = headingPID.update(headingError);

        double power = Math.min(out, 1);
        double x = Math.sin(Math.toRadians(angle)) * power;
        double y = -Math.cos(Math.toRadians(angle)) * power;
        double r = (1 + (Boolean.compare(headingError > 0, false) * -2)) * Math.min(Math.abs(headingOut),1);
        return new double[] {
                (y + x + r) * 0.3,
                (y - x - r) * 0.3,
                (y - x + r) * 0.3,
                (y + x - r) * 0.3
        };
    }

    public double[] powersToTarget (Pose2D target) {
        setTarget(target);
        return powersToTarget();
    }

    public void setTarget (Pose2D target) {this.targetPos.set(target);}

    public void toTarget () {runMotors(powersToTarget());}

    public void toTarget (Pose2D pose2d) {
        setTarget(pose2d);
        toTarget();
    }

    public int timesChecked;
    public Pose2D lastPos = new Pose2D(Double.NaN, Double.NaN, Double.NaN);

    public boolean atTarget () {
        timesChecked = Math.sqrt(
                Math.pow(currentPos.getX() - lastPos.getX(), 2) +
                Math.pow(currentPos.getY() - lastPos.getY(), 2)
                ) < 0.01 && /*Math.sqrt(
                Math.pow(currentPos[0] - targetPos.x, 2) +
                Math.pow(currentPos[1] - targetPos.y, 2)
                ) < 1 &&*/
                Math.abs(currentPos.getH() - lastPos.getH()) < 0.01 ?
                timesChecked + 1 : 0;
        lastPos.set(currentPos);
        return timesChecked > 5;
    }

    //public PathBuilder pathBuilder (Vector2D startPoint) {
        //return new PathBuilder(this, startPoint);
    //}

    public void mecanumDrive (Gamepad gamepad) {
        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double r = gamepad.right_stick_x;

        runMotors(new double[] {
                y + x + r,
                y - x - r,
                y - x + r,
                y + x - r
        });
    }

    public void trueNorthDrive (Gamepad gamepad) {
        updatePosition();

        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double r = gamepad.right_stick_x;

        double power = Math.sqrt(Math.pow(x, 2.0d) + Math.pow(y, 2.0d));

        double angle = Math.toDegrees(Math.atan2(y, x)) + 90.0d;
        angle = (angle <= 180 ? angle : angle - 360) - this.currentPos.getH();
        x = Math.sin(Math.toRadians(angle)) * power;
        y = (-Math.cos(Math.toRadians(angle))) * power;

        runMotors(new double[] {
                y + x + r,
                y - x - r,
                y - x + r,
                y + x - r
        });
    }
}