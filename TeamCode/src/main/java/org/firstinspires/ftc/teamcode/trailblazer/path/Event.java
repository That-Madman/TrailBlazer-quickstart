package org.firstinspires.ftc.teamcode.trailblazer.path;

import org.fotmrobotics.trailblazer.Vector2D;

import java.util.concurrent.Callable;

public class Event {

    private Vector2D pt;
    private Callable<Boolean> callable;

    private double range = 5;

    public Event(Vector2D pt, double range, Callable<Boolean> action) {
        this.pt = pt;
        this.range = range;
        this.callable = action;
    }

    public Event(Vector2D pt, double range, Runnable action) {
        this.pt = pt;
        this.range = range;
        this.callable = () -> {
            action.run();
            return true;
        };
    }

    public double distance(Vector2D pos) {
        return pos.minus(pt).norm();
    }

    public boolean inRange(Vector2D pos) {
        return distance(pos) < range;
    }

    public boolean call() throws Exception {
        return callable.call();
    }

    public double getRange() {
        return range;
    }

    public void setRange(double range) {
        this.range = range;
    }

    public void setPt(Vector2D pt) {
        this.pt.set(pt);
    }
}
