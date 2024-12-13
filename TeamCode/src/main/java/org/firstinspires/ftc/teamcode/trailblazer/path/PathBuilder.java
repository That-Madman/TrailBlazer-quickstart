package org.firstinspires.ftc.teamcode.trailblazer.path;


import org.firstinspires.ftc.teamcode.trailblazer.drivebase.Drive;
import org.firstinspires.ftc.teamcode.trailblazer.drivebase.MecanumDrive;
import org.fotmrobotics.trailblazer.Event;
import org.fotmrobotics.trailblazer.Spline;
import org.fotmrobotics.trailblazer.Vector2D;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.Callable;

/**
 * @author Preston Cokis
 */
public class PathBuilder {
    private Drive drive;

    double eventActivationRange = 0;

    Spline spline = new Spline(new ArrayList<>());
    List<Object> eventQueue = new ArrayList<>();
    //Byte n = 0;
    /**IDEA BOARD*/
    // TODO: Use byte list to store information on how events should be organized
    //  Maybe swap idea of taking distance for reducing grid size, this can make it more compact
    //  and efficient.
    //  First part of byte is of type
    //  Second part of byte is of position
    //  Ex. 0000 <- Segment, 0000 <- Type (Maybe)
    //  2nd byte is type
    //  Other list is of events, not including position
    //  Needed values: Point, Value (?), and Event
    //  Type is how the point is obtained
    //  Value is either a 2dvector for pos or t-value for interpolation
    //  /
    //  Combine all 'bytes' into one?
    //  00 00 0000 position type, event type, additional info (heading type)
    //  1st byte, position type (0: number, 1: Vector2D aka point, 2: t value)
    //  2nd position value (vector2d, t value, point number)
    //  3rd byte, event type (0: action, 1: heading, 2: stop)
    //  4th (optional) heading type, (0: follow, 1: constant, 2: offset)
    //  5th value

    public PathBuilder(Drive drive, Vector2D start) {
        this.drive = drive;
        spline.addPt(start);
        spline.addPt(start);
    }

    public PathBuilder(Drive mecanumDrive) {
        this.drive = mecanumDrive;
    }

    public PathBuilder pt(Vector2D point) {
        spline.addPt(point);
        return this;
    }

    // Use if using hermite spline, otherwise scrap it
    public PathBuilder pt(Vector2D point, double tangent) {
        return this;
    }

    public PathBuilder headingFollow() {
        /*
        events.add(new Event(
                Event.Type.HEADING,
                new Vector2D(0,0),
                1,
                1
        ));
         */
        eventQueue.add((byte) 0); // 00010000
        return this;
    }

    public PathBuilder headingFollow(Vector2D point) {
        eventQueue.add((byte) 0); // 01010000
        return this;
    }

    public PathBuilder headingFollow(double t) {
        eventQueue.add((byte) 0); // 10010000
        return this;
    }

    public PathBuilder headingConstant(double angle) {
        eventQueue.add((byte) 0); // 0001
        return this;
    }

    public PathBuilder headingConstant(Vector2D point, double angle) {
        eventQueue.add((byte) 0); // 1001
        return this;
    }

    public PathBuilder headingConstant(double t, double angle) {
        eventQueue.add((byte) 0); // 0101
        return this;
    }

    // TODO: Same as heading follow, but at an offset angle
    public PathBuilder headingOffset(double angle) {
        eventQueue.add((byte) 0); // 0001
        return this;
    }

    public PathBuilder headingOffset(Vector2D point, double angle) {
        eventQueue.add((byte) 0); // 0101
        return this;
    }

    public PathBuilder headingOffset(double t, double angle) {
        eventQueue.add((byte) 0); // 1001
        return this;
    }

    // TODO: Must be able to use only t values
    //  If this is implemented, it would have to be implemented for all
    public PathBuilder headingLerp(double t0, double t1, double angle0, double angle1) {
        return this;
    }

    //  TODO: Action that sets speed to zero? still not sure on this
    public PathBuilder brake() {
        eventQueue.add((byte) 0); // 00
        return this;
    }

    public PathBuilder brake(Vector2D point) {
        eventQueue.add((byte) 0); // 01
        return this;
    }

    public PathBuilder brake(double t) {
        eventQueue.add((byte) 0); // 10
        return this;
    }

    public PathBuilder wait(int milliseconds) {
        // store speed value, brake but create new timer and when timer reaches milliseconds, continue with same speed as before.
        eventQueue.add((byte) 0); // 00
        return this;
    }

    public PathBuilder wait(Vector2D point, int milliseconds) {
        eventQueue.add((byte) 0); // 01
        return this;
    }

    public PathBuilder wait(Double t, int milliseconds) {
        eventQueue.add((byte) 0); // 10
        return this;
    }

    // TODO: call action and make it change velocity parameter for velocity?
    //  the value should also be able to be negative for reversing through the path
    public PathBuilder setTranslationalVelocity(double v) {
        eventQueue.add((byte) 0); // 00
        return this;
    }

    public PathBuilder setTranslationalVelocity(Vector2D point, double v) {
        eventQueue.add((byte) 0); // 01
        return this;
    }

    public PathBuilder setTranslationalVelocity(double t, double v) {
        eventQueue.add((byte) 0); // 10
        return this;
    }

    public PathBuilder setAngularVelocity(double omega) {
        eventQueue.add((byte) 0); // 00
        return this;
    }

    public PathBuilder setAngularVelocity(Vector2D point, double omega) {
        eventQueue.add((byte) 0); // 01
        return this;
    }

    public PathBuilder setAngularVelocity(double t, double omega) {
        eventQueue.add((byte) 0); // 10
        return this;
    }

    // TODO:
    //  Use callable and return true or false
    //  when true the callable will be removed
    //  can be used to make something that runs throughout the path
    //  or runs temporarily
    public PathBuilder action(Callable action) {
        eventQueue.add((byte) 0); // 00
        return this;
    }

    public PathBuilder action(Vector2D point, Callable action) {
        eventQueue.add((byte) 0); // 01
        return this;
    }

    public PathBuilder action(Double t, Callable action) {
        eventQueue.add((byte) 0); // 10
        return this;
    }

    // TODO
    //  Create hashmap for events with each key being on a grid
    //  each event contains a vector2d position
    //  grab distance from robot pos and every event inside same grid
    //  when event in range, add callable to the queue
    //  callables will only be removed once it returns true
    //  if no value is returned, it is removed (need research)
    //  For hashmaps:
    //  If the key does not exist, assign new ArrayList<Event>()
    //  NOTE: Keep reading the first object until a marker detecting the end of list is read
    //  or if the length is 0
    public Path build() {
        //HashMap<Vector2D, ArrayList<Event>> events = new HashMap<>();
        return new Path(drive, spline);
    }
}