package org.firstinspires.ftc.teamcode.trailblazer.path;

import static org.fotmrobotics.trailblazer.PathKt.driveVector;

import org.firstinspires.ftc.teamcode.trailblazer.drivebase.Drive;
import org.fotmrobotics.trailblazer.MathKt;
import org.fotmrobotics.trailblazer.PIDF;
import org.fotmrobotics.trailblazer.Pose2D;
import org.fotmrobotics.trailblazer.Spline;
import org.fotmrobotics.trailblazer.Vector2D;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class Path {
    Drive drive;
    Spline spline = new Spline();
    ArrayList<Event> events = new ArrayList<>();
    HashMap<Event, Boolean> eventType = new HashMap<>();
    ArrayList<Event> sequentialQueue = new ArrayList<>();
    ArrayList<Event> parallelQueue = new ArrayList<>();

    PIDF translationPIDF = new PIDF(0.5, 0, 0, 0);

    enum State {
        CONTINUE,
        PAUSE,
        STOP,
        HEADING_FOLLOW,
        HEADING_OFFSET,
        HEADING_CONSTANT
    }

    State headingState = State.HEADING_FOLLOW;
    double headingValue = 0;
    double headingTarget;

    State pathState = State.CONTINUE;
    Pose2D targetPose = new Pose2D(0,0,0);

    public Path() {}

    public void finalize(Drive drive, Spline spline) {
        this.drive = drive;
        this.spline = spline;
    }

    public Path(Spline spline, HashMap<Vector2D, ArrayList<Event>> events) {
        this.spline = spline;
        //this.events = events;
    }

    public Path(Drive drive, Spline spline) {
        this.drive = drive;
        this.spline = spline;
        ArrayList<Event> eventlist = new ArrayList<>();
    }

    // TODO: for actions store them all in a hashmap with each object mapped to the corresponding grid
    public void run() {
        while (pathState != State.STOP) {
            Pose2D pos = drive.odometry.getPosition();

            Pose2D driveVector = driveVector(spline, pos, translationPIDF);

            switch (headingState) {
                case HEADING_FOLLOW:
                    headingTarget = MathKt.angleWrap(Math.toDegrees(Math.atan2(driveVector.getY(), driveVector.getX())) - 90);
                    break;
                case HEADING_OFFSET:
                    headingTarget = MathKt.angleWrap(Math.toDegrees(Math.atan2(driveVector.getY(), driveVector.getX())) - 90 + headingValue);
                    break;
                case HEADING_CONSTANT:
                    headingTarget = headingValue;
                    break;
            }

            switch (pathState) {
                case CONTINUE:
                    double t = spline.getClosestPoint(pos);
                    if (t >= 1) {
                        spline.incSegment();
                    }

                    drive.moveVector(new Pose2D(driveVector.getX(), driveVector.getY(), headingTarget), true);

                    targetPose.set(new Pose2D(pos.getX(), pos.getY(), headingTarget));
                    break;
                case PAUSE:
                    drive.movePoint(targetPose);
                    break;
            }

            for (Event event : parallelQueue) {
                try {
                    boolean condition = event.call();

                    if (condition) {parallelQueue.remove(event);}
                }

                catch (Exception e) {throw new RuntimeException(e);}
            }

            if (!sequentialQueue.isEmpty()) {
                try {
                    if (sequentialQueue.get(0).call()) {
                        sequentialQueue.remove(0);
                    }
                }

                catch (Exception e) {throw new RuntimeException(e);}
            }

            ArrayList<Event> remove = new ArrayList<>();
            for (Event event : events) {
                if (event.inRange(pos)) {
                    parallelQueue.add(event);
                    //boolean isSequential = Boolean.TRUE.equals(eventType.get(event));

                    //if (isSequential) {sequentialQueue.add(event);}
                    //else {parallelQueue.add(event);}
                    remove.add(event);
                }
            }
            events.removeAll(remove);
        }
    }
}
