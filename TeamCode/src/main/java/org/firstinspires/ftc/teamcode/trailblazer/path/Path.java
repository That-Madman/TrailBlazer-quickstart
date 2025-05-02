package org.firstinspires.ftc.teamcode.trailblazer.path;

import static org.fotmrobotics.trailblazer.PathKt.driveVector;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.trailblazer.drivebase.Drive;
import org.fotmrobotics.trailblazer.MathKt;
import org.fotmrobotics.trailblazer.PIDF;
import org.fotmrobotics.trailblazer.Pose2D;
import org.fotmrobotics.trailblazer.Spline;
import org.fotmrobotics.trailblazer.Vector2D;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * @author Preston Cokis
 */
public class Path {
    // Allows for linear OpMode methods. Important for preventing force stop mode errors.
    private LinearOpMode opMode = new LinearOpMode() {
        @Override
        public void runOpMode() throws InterruptedException {

        }
    };

    Drive drive;
    Spline spline = new Spline();

    // PIDF loop for translating back onto the spline.
    PIDF translationPIDF = new PIDF(0.5, 0, 0, 0);

    enum State {
        CONTINUE,
        PAUSE,
        STOP,
        HEADING_FOLLOW,
        HEADING_OFFSET,
        HEADING_CONSTANT
    }

    ArrayList<Event> events = new ArrayList<>();
    HashMap<Event, EventType> eventType = new HashMap<>();

    enum EventType {
        SEQUENTIAL,
        PARALLEL
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

    /**
     * Runs the path.
     */
    public void run() {
        // Resets segment to 0
        this.spline.setSegment(0);
        
        // Sets path state to continue.
        pathState = State.CONTINUE;

        // Creates a copy of the event list.
        ArrayList<Event> tempEvents = new ArrayList<>(events);

        // Event queues.
        ArrayList<Event> sequentialQueue = new ArrayList<>();
        ArrayList<Event> parallelQueue = new ArrayList<>();

        while (pathState != State.STOP) {
            // Prevents force stop mode errors.
            if (opMode.isStopRequested()) pathState = State.STOP;

            // Gets the current position.
            Pose2D pose = drive.odometry.getPosition();

            // Calculates the vector needed for translation.
            Pose2D driveVector = driveVector(spline, pose, translationPIDF);

            // Estimates how far along the segment the robot is.
            double t = spline.getClosestPoint(pose);

            // Initializes the remove list.
            ArrayList<Event> removeEvents = new ArrayList<>();
            // Loops through events.
            for (Event event : tempEvents) {
                // If the robot is past when it should run, the event gets added to the queue.
                if (event.getInterpolation() <= t && event.getSegment() == spline.getSegment()) {
                    // Checks for which queue to add the event to.
                    switch(eventType.get(event)) {
                        case PARALLEL:
                            parallelQueue.add(event);
                        case SEQUENTIAL:
                            sequentialQueue.add(event);
                    }
                    // Adds the event to the remove list.
                    removeEvents.add(event);
                }
            }
            // Removes all events that have been added to the queue.
            tempEvents.removeAll(removeEvents);

            // Initializes the remove list.
            ArrayList<Event> removeQueue = new ArrayList<>();
            // Loops through the parallel queue.
            for (Event event : parallelQueue) {
                // If the boolean returned by the callable is true, the event gets removed.
                try {
                    boolean condition = event.call();

                    if (condition) removeQueue.add(event);
                }

                catch (Exception e) {throw new RuntimeException(e);}
            }
            // Removes events that have finished running.
            parallelQueue.removeAll(removeQueue);

            // Calls the first even in the sequential queue.
            if (!sequentialQueue.isEmpty()) {
                // If the boolean returned by the callable is true, the event gets removed.
                try {
                    if (sequentialQueue.get(0).call()) {
                        sequentialQueue.remove(0);
                    }
                }

                catch (Exception e) {throw new RuntimeException(e);}
            }

            // Sets heading target.
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

            // Sets translational target.
            switch (pathState) {
                case CONTINUE:
                    // Moves the robot.
                    drive.moveVector(new Pose2D(driveVector.getX(), driveVector.getY(), headingTarget), true);

                    // Sets the target pose to be the current position. This is so the path can be paused at any point.
                    targetPose.set(new Pose2D(pose.getX(), pose.getY(), headingTarget));

                    // If the path is close to the end, set the target to the end.
                    if (t > 0.85 && spline.getLength() - 4 == spline.getSegment()) {
                        pathState = State.PAUSE;

                        Vector2D endPt = spline.getPt(spline.getLength() - 1);
                        targetPose.set(new Pose2D(endPt.getX(), endPt.getY(), headingTarget));
                    }

                    // Moves onto the next segment when the current segment has ended.
                    if (t >= 1) {
                        spline.incSegment();
                    }

                    break;
                case PAUSE:
                    // Moves the robot to a pose.
                    drive.movePoint(targetPose);

                    break;
            }
        }
    }
}
