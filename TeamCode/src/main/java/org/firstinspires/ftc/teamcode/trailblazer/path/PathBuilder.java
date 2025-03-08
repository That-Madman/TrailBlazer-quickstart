package org.firstinspires.ftc.teamcode.trailblazer.path;


import org.firstinspires.ftc.teamcode.trailblazer.drivebase.Drive;
import org.fotmrobotics.trailblazer.Pose2D;
import org.fotmrobotics.trailblazer.Spline;
import org.fotmrobotics.trailblazer.Vector2D;

import java.util.ArrayList;
import java.util.concurrent.Callable;

/**
 * Builds the path using method chaining.
 * Feel free to create your own events here. To create an event, one of the event methods must be called.
 *
 * @author Preston Cokis
 */
public class PathBuilder {
    private Path path;

    private Drive drive;

    Spline spline = new Spline(new ArrayList<>());
    ArrayList<Object> eventQueue = new ArrayList<>();
    int n = 0; // Used to track the number of points.

    /**
     * @param drive Drive base
     * @param start Starting point
     */
    public PathBuilder(Drive drive, Vector2D start) {
        this.path = new Path();
        this.drive = drive;
        spline.addPt(start);
        spline.addPt(start);
        ++n;
    }

    /*
    public PathBuilder(Drive mecanumDrive) {
        this.drive = mecanumDrive;
    }
    */

    /**
     * Adds a control point to the path.
     *
     * @param point
     */
    public PathBuilder point(Vector2D point) {
        spline.addPt(point);
        ++n;
        return this;
    }

    /**
     * Sets the heading mode to follow. The target angle will follow the direction of translation.
     *
     */
    public PathBuilder headingFollow() {
        event(() -> {path.headingState = Path.State.HEADING_FOLLOW;});
        return this;
    }

    /**
     * Sets the heading mode to follow. The target angle will follow the direction of translation.
     *
     * @param eventType Whether the event runs in parallel or sequential.
     */
    public PathBuilder headingFollow(Path.EventType eventType) {
        event(() -> {path.headingState = Path.State.HEADING_FOLLOW;}, eventType);
        return this;
    }

    /**
     * Sets the heading mode to follow. The target angle will follow the direction of translation.
     *
     * @param t How far along the segment the event runs.
     */
    public PathBuilder headingFollow(double t) {
        event(t, () -> {path.headingState = Path.State.HEADING_FOLLOW;});
        return this;
    }

    /**
     * Sets the heading mode to follow. The target angle will follow the direction of translation.
     *
     * @param t How far along the segment the event runs.
     * @param eventType Whether the event runs in parallel or sequential.
     */
    public PathBuilder headingFollow(double t, Path.EventType eventType) {
        event(t, () -> {path.headingState = Path.State.HEADING_FOLLOW;}, eventType);
        return this;
    }

    /**
     * Sets the heading mode to constant. The target angle will be constant.
     *
     * @param angle Target angle.
     */
    public PathBuilder headingConstant(double angle) {
        event(() -> {
            path.headingState = Path.State.HEADING_CONSTANT;
            path.headingValue = angle;
        });
        return this;
    }

    /**
     * Sets the heading mode to constant. The target angle will be constant.
     *
     * @param angle Target angle.
     * @param eventType Whether the event runs in parallel or sequential.
     */
    public PathBuilder headingConstant(double angle, Path.EventType eventType) {
        event(() -> {
            path.headingState = Path.State.HEADING_CONSTANT;
            path.headingValue = angle;
        }, eventType);
        return this;
    }

    /**
     * Sets the heading mode to constant. The target angle will be constant.
     *
     * @param t How far along the segment the event runs
     * @param angle Target angle
     */
    public PathBuilder headingConstant(double t, double angle) {
        event(t, () -> {
            path.headingState = Path.State.HEADING_CONSTANT;
            path.headingValue = angle;
        });
        return this;
    }

    /**
     * Sets the heading mode to constant. The target angle will be constant.
     *
     * @param t How far along the segment the event runs
     * @param angle Target angle
     * @param eventType Whether the event runs in parallel or sequential
     */
    public PathBuilder headingConstant(double t, double angle, Path.EventType eventType) {
        event(t, () -> {
            path.headingState = Path.State.HEADING_CONSTANT;
            path.headingValue = angle;
        }, eventType);
        return this;
    }

    /**
     * Sets the heading mode to offset. The target angle will follow the path at an offset.
     *
     * @param angle Angle offset
     */
    public PathBuilder headingOffset(double angle) {
        event(() -> {
            path.headingState = Path.State.HEADING_OFFSET;
            path.headingValue = angle;
        });
        return this;
    }

    /**
     * Sets the heading mode to offset. The target angle will follow the path at an offset.
     *
     * @param angle Angle offset
     * @param eventType Whether the event runs in parallel or sequential
     */
    public PathBuilder headingOffset(double angle, Path.EventType eventType) {
        event(() -> {
            path.headingState = Path.State.HEADING_OFFSET;
            path.headingValue = angle;
        }, eventType);
        return this;
    }

    /**
     * Sets the heading mode to offset. The target angle will follow the path at an offset.
     *
     * @param t How far along the segment the event runs
     * @param angle Angle offset
     */
    public PathBuilder headingOffset(double t, double angle) {
        event(t, () -> {
            path.headingState = Path.State.HEADING_OFFSET;
            path.headingValue = angle;
        });
        return this;
    }

    /**
     * Sets the heading mode to offset. The target angle will follow the path at an offset.
     *
     * @param t How far along the segment the event runs
     * @param angle Angle offset
     * @param eventType Whether the event runs in parallel or sequential
     */
    public PathBuilder headingOffset(double t, double angle, Path.EventType eventType) {
        event(t, () -> {
            path.headingState = Path.State.HEADING_OFFSET;
            path.headingValue = angle;
        }, eventType);
        return this;
    }

    /**
     * Switches the path state to pause. The target becomes a set point.
     *
     * @param condition If true, resume the path, otherwise, stay paused
     */
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

    /**
     * Switches the path state to pause. The target becomes a set point.
     *
     * @param condition If true, resume the path, otherwise, stay paused
     * @param eventType Whether the event runs in parallel or sequential
     */
    public PathBuilder pause(Callable<Boolean> condition, Path.EventType eventType) {
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
        }, eventType);
        return this;
    }

    /**
     * Switches the path state to pause. The target becomes a set point.
     *
     * @param t How far along the segment the event runs
     * @param condition If true, resume the path, otherwise, stay paused
     */
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

    /**
     * Switches the path state to pause. The target becomes a set point.
     *
     * @param t How far along the segment the event runs
     * @param condition If true, resume the path, otherwise, stay paused
     * @param eventType Whether the event runs in parallel or sequential
     */
    public PathBuilder pause(double t, Callable<Boolean> condition, Path.EventType eventType) {
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
        }, eventType);
        return this;
    }

    /**
     * Switches the path state to pause. The target becomes a set point.
     *
     * @param pose The target pose
     * @param condition If true, resume the path, otherwise, stay paused
     */
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

    /**
     * Switches the path state to pause. The target becomes a set point.
     *
     * @param pose The target pose
     * @param condition If true, resume the path, otherwise, stay paused
     * @param eventType Whether the event runs in parallel or sequential
     */
    public PathBuilder toPose(Pose2D pose, Callable<Boolean> condition, Path.EventType eventType) {
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
        }, eventType);
        return this;
    }

    /**
     * Switches the path state to pause. The target becomes a set point.
     *
     * @param t How far along the segment the event runs
     * @param pose The target pose
     * @param condition If true, resume the path, otherwise, stay paused
     */
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

    /**
     * Switches the path state to pause. The target becomes a set point.
     *
     * @param t How far along the segment the event runs
     * @param pose The target pose
     * @param condition If true, resume the path, otherwise, stay paused
     * @param eventType Whether the event runs in parallel or sequential
     */
    public PathBuilder toPose(double t, Pose2D pose, Callable<Boolean> condition, Path.EventType eventType) {
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
        }, eventType);
        return this;
    }

    /**
     * Changes the speed for left/right.
     *
     * @param scale Speed scale
     */
    public PathBuilder xScale(double scale) {
        event(() -> {drive.xScale = scale;});
        return this;
    }

    /**
     * Changes the speed for left/right.
     *
     * @param scale Speed scale
     * @param eventType Whether the event runs in parallel or sequential
     */
    public PathBuilder xScale(double scale, Path.EventType eventType) {
        event(() -> {drive.xScale = scale;}, eventType);
        return this;
    }

    /**
     * Changes the speed for left/right.
     *
     * @param t How far along the segment the event runs
     * @param scale Speed scale
     */
    public PathBuilder xScale(Double t, double scale) {
        event(t, () -> {drive.xScale = scale;});
        return this;
    }

    /**
     * Changes the speed for left/right.
     *
     * @param t How far along the segment the event runs
     * @param scale Speed scale
     * @param eventType Whether the event runs in parallel or sequential
     */
    public PathBuilder xScale(Double t, double scale, Path.EventType eventType) {
        event(t, () -> {drive.xScale = scale;}, eventType);
        return this;
    }

    /**
     * Changes the speed for forward/backward.
     *
     * @param scale Speed scale
     */
    public PathBuilder yScale(double scale) {
        event(() -> {drive.yScale = scale;});
        return this;
    }

    /**
     * Changes the speed for forward/backward.
     *
     * @param scale Speed scale
     * @param eventType Whether the event runs in parallel or sequential
     */
    public PathBuilder yScale(double scale, Path.EventType eventType) {
        event(() -> {drive.yScale = scale;}, eventType);
        return this;
    }

    /**
     * Changes the speed for forward/backward.
     *
     * @param t How far along the segment the event runs
     * @param scale Speed scale
     */
    public PathBuilder yScale(Double t, double scale) {
        event(t, () -> {drive.yScale = scale;});
        return this;
    }

    /**
     * Changes the speed for forward/backward.
     *
     * @param t How far along the segment the event runs
     * @param scale Speed scale
     * @param eventType Whether the event runs in parallel or sequential
     */
    public PathBuilder yScale(Double t, double scale, Path.EventType eventType) {
        event(t, () -> {drive.yScale = scale;}, eventType);
        return this;
    }

    /**
     * Changes the speed for forward/backward/left/right.
     *
     * @param scale Speed scale
     */
    public PathBuilder translationalScale(double scale) {
        event(() -> {
            drive.xScale = scale;
            drive.yScale = scale;
        });
        return this;
    }

    /**
     * Changes the speed for forward/backward/left/right.
     *
     * @param scale Speed scale
     * @param eventType Whether the event runs in parallel or sequential
     */
    public PathBuilder translationalScale(double scale, Path.EventType eventType) {
        event(() -> {
            drive.xScale = scale;
            drive.yScale = scale;
        }, eventType);
        return this;
    }

    /**
     * Changes the speed for forward/backward/left/right.
     *
     * @param t How far along the segment the event runs
     * @param scale Speed scale
     */
    public PathBuilder translationalScale(Double t, double scale) {
        event(t, () -> {
            drive.xScale = scale;
            drive.yScale = scale;
        });
        return this;
    }

    /**
     * Changes the speed for forward/backward/left/right.
     *
     * @param t How far along the segment the event runs
     * @param scale Speed scale
     * @param eventType Whether the event runs in parallel or sequential
     */
    public PathBuilder translationalScale(Double t, double scale, Path.EventType eventType) {
        event(t, () -> {
            drive.xScale = scale;
            drive.yScale = scale;
        }, eventType);
        return this;
    }

    /**
     * Changes the speed for rotating.
     *
     * @param scale Speed scale
     */
    public PathBuilder angularScale(double scale) {
        event(() -> {drive.angularScale = scale;});
        return this;
    }

    /**
     * Changes the speed for rotating.
     *
     * @param scale Speed scale
     * @param eventType Whether the event runs in parallel or sequential
     */
    public PathBuilder angularScale(double scale, Path.EventType eventType) {
        event(() -> {drive.angularScale = scale;}, eventType);
        return this;
    }

    /**
     * Changes the speed for rotating.
     *
     * @param t How far along the segment the event runs
     * @param scale Speed scale
     */
    public PathBuilder angularScale(Double t, double scale) {
        event(t, () -> {drive.angularScale = scale;});
        return this;
    }

    /**
     * Changes the speed for rotating.
     *
     * @param t How far along the segment the event runs
     * @param scale Speed scale
     * @param eventType Whether the event runs in parallel or sequential
     */
    public PathBuilder angularScale(Double t, double scale, Path.EventType eventType) {
        event(t, () -> {drive.angularScale = scale;}, eventType);
        return this;
    }

    /**
     * Runs a code segment.
     *
     * @param action Callable with a boolean return value
     */
    public PathBuilder action(Callable<Boolean> action) {
        event(action);
        return this;
    }

    /**
     * Runs a code segment.
     *
     * @param action Callable with a boolean return value
     * @param eventType Whether the event runs in parallel or sequential
     */
    public PathBuilder action(Callable<Boolean> action, Path.EventType eventType) {
        event(action, eventType);
        return this;
    }

    /**
     * Runs a code segment.
     *
     * @param action Runnable
     */
    public PathBuilder action(Runnable action) {
        event(action);
        return this;
    }

    /**
     * Runs a code segment.
     *
     * @param action Runnable
     * @param eventType Whether the event runs in parallel or sequential
     */
    public PathBuilder action(Runnable action, Path.EventType eventType) {
        event(action, eventType);
        return this;
    }

    /**
     * Runs a code segment.
     *
     * @param t How far along the segment the event runs
     * @param action Callable with a boolean return value
     */
    public PathBuilder action(Double t, Callable<Boolean> action) {
        event(t, action);
        return this;
    }

    /**
     * Runs a code segment.
     *
     * @param t How far along the segment the event runs
     * @param action Callable with a boolean return value
     * @param eventType Whether the event runs in parallel or sequential
     */
    public PathBuilder action(Double t, Callable<Boolean> action, Path.EventType eventType) {
        event(t, action, eventType);
        return this;
    }

    /**
     * Runs a code segment.
     *
     * @param t How far along the segment the event runs
     * @param action Runnable
     */
    public PathBuilder action(Double t, Runnable action) {
        event(t, action);
        return this;
    }

    /**
     * Runs a code segment.
     *
     * @param t How far along the segment the event runs
     * @param action Runnable
     * @param eventType Whether the event runs in parallel or sequential
     */
    public PathBuilder action(Double t, Runnable action, Path.EventType eventType) {
        event(t, action, eventType);
        return this;
    }

    private void event(Callable<Boolean> event) {
        eventQueue.add(new Event(Integer.MIN_VALUE, 0, event));
        eventQueue.add(n);
        eventQueue.add(Path.EventType.PARALLEL);
    }

    private void event(Callable<Boolean> event, Path.EventType eventType) {
        eventQueue.add(new Event(Integer.MIN_VALUE, 0, event));
        eventQueue.add(n);
        eventQueue.add(eventType);
    }

    private void event(Runnable event) {
        eventQueue.add(new Event(Integer.MIN_VALUE, 0, event));
        eventQueue.add(n);
        eventQueue.add(Path.EventType.PARALLEL);
    }

    private void event(Runnable event, Path.EventType eventType) {
        eventQueue.add(new Event(Integer.MIN_VALUE, 0, event));
        eventQueue.add(n);
        eventQueue.add(eventType);
    }

    public void event(Double t, Callable<Boolean> event) {
        eventQueue.add(new Event(Integer.MIN_VALUE, t, event));
        eventQueue.add(n);
        eventQueue.add(Path.EventType.PARALLEL);
    }

    public void event(Double t, Callable<Boolean> event, Path.EventType eventType) {
        eventQueue.add(new Event(Integer.MIN_VALUE, t, event));
        eventQueue.add(n);
        eventQueue.add(eventType);
    }

    public void event(Double t, Runnable event) {
        eventQueue.add(new Event(Integer.MIN_VALUE, t, event));
        eventQueue.add(n);
        eventQueue.add(Path.EventType.PARALLEL);
    }

    public void event(Double t, Runnable event, Path.EventType eventType) {
        eventQueue.add(new Event(Integer.MIN_VALUE, t, event));
        eventQueue.add(n);
        eventQueue.add(eventType);
    }

    /**
     * Finalizes the path. This method must be called in order to create the path.
     *
     * @return Path
     */
    public Path build() {
        // Gets the end point.
        Vector2D endPt = spline.getPt(spline.getLength()-1);
        // Adds the end point to the end so the last segment can be formed.
        spline.addPt(endPt);

        // Checks for when the path is reaching the end. When at the end, the path stops.
        event(0.85, () -> {
            if (drive.atTarget()) path.pathState = Path.State.STOP;

            return false;
        });

        // Adds in all events to the path.
        while (!eventQueue.isEmpty()) {
            // Gets the event.
            Event event = (Event) eventQueue.get(0);

            // Modifies the event to have the correct values.
            int segment = Math.min(Math.max(0, (int) eventQueue.get(1) - 1), spline.getLength() - 4);
            event.setSegment(segment);

            // Adds the event to the path.
            path.events.add(event);

            // Adds in the event type to a hashmap with the key as the event and type as the value.
            path.eventType.put(event, (Path.EventType) eventQueue.get(2));

            // Clears the segment of the event queue.
            eventQueue.subList(0, 3).clear();
        }

        // Finishing touches
        path.finalize(drive, spline);

        return path;
    }
}