package org.firstinspires.ftc.teamcode.trailblazer.path;

import java.util.concurrent.Callable;

/**
 * @author Preston Cokis
 */
public class Event {

    private int segment;
    private double t;

    private Callable<Boolean> callable;

    /**
     * @param segment Path segment the event belongs to
     * @param t How far along the segment
     * @param action Callable with a boolean return value
     */
    public Event(Integer segment, double t, Callable<Boolean> action) {
        this.segment = segment;
        this.t = t;
        this.callable = action;
    }

    /**
     * @param segment Path segment the event belongs to
     * @param t How far along the segment
     * @param action Runnable
     */
    public Event(Integer segment, double t, Runnable action) {
        this.segment = segment;
        this.t = t;
        this.callable = () -> {
            action.run();
            return true;
        };
    }

    /**
     * Calls the callable.
     *
     * @return Boolean returned by the callable
     */
    public boolean call() throws Exception {
        return callable.call();
    }

    public int getSegment() {
        return segment;
    }

    public void setSegment(int segment) {
        this.segment = segment;
    }

    public double getInterpolation() {
        return t;
    }

    public void setInterpolation(double t) {
        this.t = t;
    }
}
