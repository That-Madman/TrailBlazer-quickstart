package org.firstinspires.ftc.teamcode.trailblazer.path;

import static org.fotmrobotics.trailblazer.PathKt.driveVector;

import org.firstinspires.ftc.teamcode.trailblazer.drivebase.Drive;
import org.fotmrobotics.trailblazer.Event;
import org.fotmrobotics.trailblazer.PIDF;
import org.fotmrobotics.trailblazer.Pose2D;
import org.fotmrobotics.trailblazer.Spline;
import org.fotmrobotics.trailblazer.Vector2D;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class Path {
    //List<Vector2D> controlPoints = new ArrayList<>();
    //List<Event> events = new ArrayList<>();
    Drive drive;
    Spline spline = new Spline();
    HashMap<Vector2D, ArrayList<Event>> events = new HashMap<>();
    List<Event> queue = new ArrayList<>();

    PIDF translationPIDF = new PIDF(0.5, 0, 0, 0);

    int segment = 0;
    Event.Heading headingType = Event.Heading.FOLLOW;

    boolean running = false;

    public Path(Spline spline, HashMap<Vector2D, ArrayList<Event>> events) {
        this.spline = spline;
        this.events = events;
    }

    public Path(Drive drive, Spline spline) {
        this.drive = drive;
        this.spline = spline;
    }

    // TODO: for actions store them all in a hashmap with each object mapped to the corresponding grid
    public void run() {
        //this.segment = 0;
        headingType = Event.Heading.FOLLOW;
        double headingTarget = 0;
        running = true;
        while (running) {
            //update();
            Pose2D pos = drive.odometry.getPosition();

            //Vector2D grid = new Vector2D(pos.getX() % 12, pos.getY() % 12);

            Pose2D driveVector = driveVector(spline, pos, translationPIDF);

            headingTarget = Math.toDegrees(driveVector.getH()) - 90;
            //headingTarget = Math.toDegrees(Math.atan2(driveVector.getY(), driveVector.getX()));
            //headingTarget = headingTarget <= 180 ? headingTarget : headingTarget + 360;
            //headingTarget -= 90;
            /*
            switch(headingType) {
                case FOLLOW:
                    headingTarget = Math.toDegrees(Math.atan2(driveVector.getY(), driveVector.getX()));
                    headingTarget = headingTarget <= 180 ? headingTarget : headingTarget - 360;
                case CONSTANT:
                    continue;
                case OFFSET:
                    continue;
            }
             */

            drive.moveVector(driveVector, headingTarget);

            double t = spline.getClosestPoint(pos);
            if (t >= 1) {
                spline.incSegment();
            } //else if (t <= 0) {
                //spline.decSegment();
            //}

            if (spline.getLength() - 1 == spline.getSegment() && t > 0.9) {
                drive.movePoint(spline.getPt(-1), headingTarget);
            }

            // Get robot position
            // Robot position % 12
            // Look for key in hashmap
            // Look at event position
            // If the distance between event and robot is close, add event to queue

            /*
            for (Event event : events[grid]) {
                double distance = (event.pos - drive.getPosition()).norm();
                if (distance < 1) {
                    queue.add(event);
                }
            }

            for (Event event : queue) {
                // TODO: Every event must have a runnable and condition (Callable). If there is no condition,
                //  delete the event from the queue after it is done.
            }
            */
        }
    }

    public void update() {
        //double t = spline.getClosestPoint(drive.getPosition());
        Pose2D driveVector = driveVector(spline, drive.odometry.getPosition(), translationPIDF);

    }

    public Pose2D test() {
        return driveVector(spline, drive.odometry.getPosition(), translationPIDF);
    }

    public void stop() {
        running = false;
    }

    public void loop() {

    }
}
