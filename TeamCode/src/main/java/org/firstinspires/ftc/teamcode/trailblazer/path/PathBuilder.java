package org.firstinspires.ftc.teamcode.trailblazer.path;


import org.firstinspires.ftc.teamcode.trailblazer.drivebase.Drive;
import org.fotmrobotics.trailblazer.Pose2D;
import org.fotmrobotics.trailblazer.Spline;
import org.fotmrobotics.trailblazer.Vector2D;

import java.util.ArrayList;
import java.util.concurrent.Callable;

/**
 * @author Preston Cokis
 */
public class PathBuilder {
    private Path path;

    private Drive drive;

    double eventRange = 5;

    Spline spline = new Spline(new ArrayList<>());
    ArrayList<Object> eventQueue = new ArrayList<>();
    int n = 0;

    public PathBuilder(Drive drive, Vector2D start) {
        this.path = new Path();
        this.drive = drive;
        spline.addPt(start);
        spline.addPt(start);
        ++n;
    }

    public PathBuilder(Drive mecanumDrive) {
        this.drive = mecanumDrive;
    }

    public PathBuilder pt(Vector2D point) {
        spline.addPt(point);
        ++n;
        return this;
    }

    public PathBuilder setEventRange(double range) {
        eventRange = range;
        return this;
    }

    public PathBuilder headingFollow() {
        event(() -> {path.headingState = Path.State.HEADING_FOLLOW;});
        return this;
    }

    public PathBuilder headingFollow(Vector2D point) {
        event(point, () -> {path.headingState = Path.State.HEADING_FOLLOW;});
        return this;
    }

    public PathBuilder headingFollow(double t) {
        event(t, () -> {path.headingState = Path.State.HEADING_FOLLOW;});
        return this;
    }

    public PathBuilder headingConstant(double angle) {
        event(() -> {
            path.headingState = Path.State.HEADING_CONSTANT;
            path.headingValue = angle;
        });
        return this;
    }

    public PathBuilder headingConstant(Vector2D point, double angle) {
        event(point, () -> {
            path.headingState = Path.State.HEADING_CONSTANT;
            path.headingValue = angle;
        });
        return this;
    }

    public PathBuilder headingConstant(double t, double angle) {
        event(t, () -> {
            path.headingState = Path.State.HEADING_CONSTANT;
            path.headingValue = angle;
        });
        return this;
    }

    public PathBuilder headingOffset(double angle) {
        event(() -> {
            path.headingState = Path.State.HEADING_OFFSET;
            path.headingValue = angle;
        });
        return this;
    }

    public PathBuilder headingOffset(Vector2D point, double angle) {
        event(point, () -> {
            path.headingState = Path.State.HEADING_OFFSET;
            path.headingValue = angle;
        });
        return this;
    }

    public PathBuilder headingOffset(double t, double angle) {
        event(t, () -> {
            path.headingState = Path.State.HEADING_OFFSET;
            path.headingValue = angle;
        });
        return this;
    }

    public PathBuilder pause(Callable<Boolean> condition) {
        event(() -> {
            path.pathState = Path.State.PAUSE;
            try {
                if (condition.call()) {
                    path.pathState = Path.State.CONTINUE;
                    return true;
                }
                return false;
            }
            catch (Exception e) {throw new RuntimeException(e);}
        });
        return this;
    }

    public PathBuilder pause(Vector2D point, Callable<Boolean> condition) {
        event(point, () -> {
            path.pathState = Path.State.PAUSE;
            try {
                if (condition.call()) {
                    path.pathState = Path.State.CONTINUE;
                    return true;
                }
                return false;
            }
            catch (Exception e) {throw new RuntimeException(e);}
        });
        return this;
    }

    public PathBuilder pause(double t, Callable<Boolean> condition) {
        event(t, () -> {
            path.pathState = Path.State.PAUSE;
            try {
                if (condition.call()) {
                    path.pathState = Path.State.CONTINUE;
                    return true;
                }
                return false;
            }
            catch (Exception e) {throw new RuntimeException(e);}
        });
        return this;
    }

    public PathBuilder toPose(Pose2D pose, Callable<Boolean> condition) {
        event(() -> {
            path.pathState = Path.State.PAUSE;
            path.targetPose = pose;
            try {
                if (condition.call()) {
                    path.targetPose = pose;
                    path.pathState = Path.State.CONTINUE;
                    return true;
                }
                return false;
            }
            catch (Exception e) {throw new RuntimeException(e);}
        });
        return this;
    }

    public PathBuilder toPose(Vector2D point, Pose2D pose, Callable<Boolean> condition) {
        event(point, () -> {
            path.pathState = Path.State.PAUSE;
            path.targetPose = pose;
            try {
                if (condition.call()) {
                    path.targetPose = pose;
                    path.pathState = Path.State.CONTINUE;
                    return true;
                }
                return false;
            }
            catch (Exception e) {throw new RuntimeException(e);}
        });
        return this;
    }

    public PathBuilder toPose(double t, Pose2D pose, Callable<Boolean> condition) {
        event(t, () -> {
            path.pathState = Path.State.PAUSE;
            path.targetPose = pose;
            try {
                if (condition.call()) {
                    path.pathState = Path.State.CONTINUE;
                    return true;
                }
                return false;
            }
            catch (Exception e) {throw new RuntimeException(e);}
        });
        return this;
    }

    public PathBuilder xScale(double scale) {
        if (scale < 0 || scale > 1) throw new RuntimeException("Scale must be between 0 and 1.");
        event(() -> {drive.xScale = scale;});
        return this;
    }

    public PathBuilder xScale(Vector2D point, double scale) {
        if (scale < 0 || scale > 1) throw new RuntimeException("Scale must be between 0 and 1.");
        event(point, () -> {drive.xScale = scale;});
        return this;
    }

    public PathBuilder xScale(Double t, double scale) {
        if (scale < 0 || scale > 1) throw new RuntimeException("Scale must be between 0 and 1.");
        event(t, () -> {drive.xScale = scale;});
        return this;
    }

    public PathBuilder yScale(double scale) {
        if (scale < 0 || scale > 1) throw new RuntimeException("Scale must be between 0 and 1.");
        event(() -> {drive.yScale = scale;});
        return this;
    }

    public PathBuilder yScale(Vector2D point, double scale) {
        if (scale < 0 || scale > 1) throw new RuntimeException("Scale must be between 0 and 1.");
        event(point, () -> {drive.yScale = scale;});
        return this;
    }

    public PathBuilder yScale(Double t, double scale) {
        if (scale < 0 || scale > 1) throw new RuntimeException("Scale must be between 0 and 1.");
        event(t, () -> {drive.yScale = scale;});
        return this;
    }

    public PathBuilder translationalScale(double scale) {
        if (scale < 0 || scale > 1) throw new RuntimeException("Scale must be between 0 and 1.");
        event(() -> {
            drive.xScale = scale;
            drive.yScale = scale;
        });
        return this;
    }

    public PathBuilder translationalScale(Vector2D point, double scale) {
        if (scale < 0 || scale > 1) throw new RuntimeException("Scale must be between 0 and 1.");
        event(point, () -> {
            drive.xScale = scale;
            drive.yScale = scale;
        });
        return this;
    }

    public PathBuilder translationalScale(Double t, double scale) {
        if (scale < 0 || scale > 1) throw new RuntimeException("Scale must be between 0 and 1.");
        event(t, () -> {
            drive.xScale = scale;
            drive.yScale = scale;
        });
        return this;
    }

    public PathBuilder angularScale(double scale) {
        if (scale < 0 || scale > 1) throw new RuntimeException("Scale must be between 0 and 1.");
        event(() -> {drive.angularScale = scale;});
        return this;
    }

    public PathBuilder angularScale(Vector2D point, double scale) {
        if (scale < 0 || scale > 1) throw new RuntimeException("Scale must be between 0 and 1.");
        event(point, () -> {drive.angularScale = scale;});
        return this;
    }

    public PathBuilder angularScale(Double t, double scale) {
        if (scale < 0 || scale > 1) throw new RuntimeException("Scale must be between 0 and 1.");
        event(t, () -> {drive.angularScale = scale;});
        return this;
    }

    public PathBuilder action(Callable<Boolean> action) {
        event(action);
        return this;
    }

    public PathBuilder action(Vector2D point, Callable<Boolean> action) {
        event(point, action);
        return this;
    }

    public PathBuilder action(Double t, Callable<Boolean> action) {
        event(t, action);
        return this;
    }

    private void event(Callable<Boolean> event) {
        eventQueue.add(new Event(new Vector2D(Double.NaN, Double.NaN), eventRange, event));
        eventQueue.add(0x00);
        eventQueue.add(n);
    }

    private void event(Runnable event) {
        eventQueue.add(new Event(new Vector2D(Double.NaN, Double.NaN), eventRange, event));
        eventQueue.add(0x00);
        eventQueue.add(n);
    }

    public void event(Vector2D point, Callable<Boolean> event) {
        eventQueue.add(new Event(point, eventRange, event));
        eventQueue.add(0x01);
    }

    public void event(Vector2D point, Runnable event) {
        eventQueue.add(new Event(point, eventRange, event));
        eventQueue.add(0x01);
    }

    public void event(Double t, Callable<Boolean> event) {
        eventQueue.add(new Event(new Vector2D(Double.NaN, Double.NaN), eventRange, event));
        eventQueue.add(0x02);
        eventQueue.add(n);
        eventQueue.add(t);
    }

    public void event(Double t, Runnable event) {
        eventQueue.add(new Event(new Vector2D(Double.NaN, Double.NaN), eventRange, event));
        eventQueue.add(0x02);
        eventQueue.add(n);
        eventQueue.add(t);
    }

    public Path build() {
        Vector2D endPt = spline.getPt(spline.getLength()-1);
        spline.addPt(endPt);

        while (!eventQueue.isEmpty()) {
            Event event = (Event) eventQueue.get(0);

            switch ((int) eventQueue.get(1) & 0x03) {
                case 0:
                    event.setPt(spline.getPt((int) eventQueue.get(2)));
                    path.events.add(event);

                    eventQueue.subList(0, 3).clear();
                    break;
                case 1:
                    path.events.add(event);

                    eventQueue.subList(0, 2).clear();
                    break;
                case 2:
                    int segment = Math.min(Math.max(0, (int) eventQueue.get(2) - 1), spline.getLength() - 4);
                    event.setPt(spline.getPoint(segment, (double) eventQueue.get(3)));
                    path.events.add(event);

                    eventQueue.subList(0, 4).clear();
                    break;
            }
        }

        path.events.add(new Event(endPt, 5, () -> {
            path.pathState = Path.State.PAUSE;
            path.targetPose.setX(endPt.getX());
            path.targetPose.setY(endPt.getY());

            if (drive.atTarget()) path.pathState = Path.State.STOP;

            return false;
        }));

        path.finalize(drive, spline);

        return path;
    }
}