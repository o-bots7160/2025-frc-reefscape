package frc.robot.motion;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

/**
 * Helper/utility that encapsulates trapezoidal motion profiling state management (goal, current profiled state, interruption/braking, overshoot
 * handling) separate from any particular mechanism implementation.
 * <p>
 * The intent is to remove this lifecycle complexity from subsystems so they can focus on hardware concerns and feedforward/PID calculations while
 * still reusing consistent profile + reversal safeguards.
 */
public class TrapezoidalMotionManager {

    public static class AdvanceSnapshot {
        public final State   previousProfiled;

        public final State   measured;

        public final boolean overshootReanchored;

        private AdvanceSnapshot(State previousProfiled, State measured, boolean overshootReanchored) {
            this.previousProfiled    = previousProfiled;
            this.measured            = measured;
            this.overshootReanchored = overshootReanchored;
        }
    }

    private final double           minimumSetPoint;

    private final double           maximumSetPoint;

    private final double           setPointTolerance;

    private final double           stoppingTolerance;

    private final double           maximumAcceleration;

    private final TrapezoidProfile profile;

    private TrapezoidProfile.State goalState        = new TrapezoidProfile.State();

    private TrapezoidProfile.State nextState        = new TrapezoidProfile.State(0, 0);

    private TrapezoidProfile.State previousState    = new TrapezoidProfile.State(0, 0);

    private double                 lastAcceleration = 0.0;

    private double                 lastMeasuredPos  = 0.0;

    private double                 lastMeasuredVel  = 0.0;

    public TrapezoidalMotionManager(double minimumSetPoint, double maximumSetPoint, double setPointTolerance,
            double stoppingTolerance, double maximumVelocity, double maximumAcceleration) {
        this.minimumSetPoint     = minimumSetPoint;
        this.maximumSetPoint     = maximumSetPoint;
        this.setPointTolerance   = setPointTolerance;
        this.stoppingTolerance   = stoppingTolerance;
        this.maximumAcceleration = maximumAcceleration;
        this.profile             = new TrapezoidProfile(new TrapezoidProfile.Constraints(maximumVelocity, maximumAcceleration));
    }

    public void setTarget(double desiredPosition, double measuredPos, double measuredVel) {
        double clamped = clamp(desiredPosition);
        nextState = new State(measuredPos, measuredVel);
        goalState = new State(clamped, 0.0);
    }

    public void setTargetVelocity(double desiredVelocity, double measuredPos, double currentProfileVel) {
        // distance needed to change velocity at max accel (v1^2 - v2^2)/(2a)
        double base         = (Math.pow(currentProfileVel, 2.0) - Math.pow(desiredVelocity, 2.0)) / (2.0 * maximumAcceleration);
        double displacement = currentProfileVel > desiredVelocity ? base : -base;
        setTarget(measuredPos + displacement, measuredPos, currentProfileVel);
    }

    public void interrupt(double measuredPos, double measuredVel) {
        double originalGoal    = goalState.position;
        double brakingDistance = brakingDistance(measuredVel);
        double newGoal         = measuredPos;                 // default hold
        if (measuredVel > 0) {
            if (measuredPos < originalGoal) {
                newGoal = Math.min(measuredPos + brakingDistance, originalGoal);
            }
        } else if (measuredVel < 0) {
            if (measuredPos > originalGoal) {
                newGoal = Math.max(measuredPos - brakingDistance, originalGoal);
            }
        }
        goalState = new State(clamp(newGoal), 0.0);
        nextState = new State(measuredPos, measuredVel);
    }

    /**
     * Advance the internal profile by dt seconds based on the current measured state. Handles overshoot reversal anchoring.
     *
     * @param measuredPos current measured mechanism position
     * @param measuredVel current measured mechanism velocity
     * @param dt          loop period
     * @return snapshot with measured + previous profiled states and whether overshoot anchor occurred
     */
    public AdvanceSnapshot advance(double measuredPos, double measuredVel, double dt) {
        previousState   = nextState;
        lastMeasuredPos = measuredPos;
        lastMeasuredVel = measuredVel;

        // overshoot / reversal handling
        double  posError   = goalState.position - measuredPos;                               // desired - actual
        boolean movingAway = (Math.abs(posError) > setPointTolerance) && (measuredVel != 0.0)
                && (Math.signum(measuredVel) != Math.signum(posError));
        boolean reanchored = false;
        State   startState = new State(measuredPos, measuredVel);

        if (movingAway) {
            // re-anchor with zero velocity to allow clean reversal
            startState = new State(measuredPos, 0.0);
            nextState  = startState;
            reanchored = true;
        }
        nextState        = profile.calculate(dt, startState, goalState);
        lastAcceleration = (nextState.velocity - previousState.velocity) / dt;
        return new AdvanceSnapshot(previousState, startState, reanchored);
    }

    public boolean atTarget(double measuredPos, double measuredVel) {
        double  posError  = measuredPos - goalState.position;
        boolean withinPos = Math.abs(posError) < setPointTolerance;
        boolean withinVel = Math.abs(measuredVel) < stoppingTolerance;
        return withinPos && withinVel;
    }

    public void lockAtGoal() {
        goalState = new State(goalState.position, 0.0);
        nextState = new State(goalState.position, 0.0);
    }

    public State getGoalState() {
        return goalState;
    }

    public State getNextState() {
        return nextState;
    }

    public State getPreviousState() {
        return previousState;
    }

    public double getLastAcceleration() {
        return lastAcceleration;
    }

    public double getLastMeasuredPos() {
        return lastMeasuredPos;
    }

    public double getLastMeasuredVel() {
        return lastMeasuredVel;
    }

    private double brakingDistance(double velocity) {
        double v = Math.abs(velocity);
        return (v * v) / (2.0 * maximumAcceleration);
    }

    private double clamp(double position) {
        if (position < minimumSetPoint)
            return minimumSetPoint;
        if (position > maximumSetPoint)
            return maximumSetPoint;
        return position;
    }
}
