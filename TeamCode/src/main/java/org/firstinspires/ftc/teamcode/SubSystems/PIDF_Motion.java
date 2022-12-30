package org.firstinspires.ftc.teamcode.SubSystems;

public class PIDF_Motion {

    public class PIDController {

//    public static double Kp = 0.0;
//    public static double Ki = 0.0;
//    public static double Kd = 0.0;

        double Kp = 0.0;
        double Ki = 0.0;
        double Kd = 0.0;

        /* Derivative low-pass filter time constant */
        double tau;

        /* Output limits */
        double limMin = -1.0;
        double limMax = 1.0;

        /* Integrator limits */
        double limMinInt;
        double limMaxInt;

        /* Sample time (in seconds) */
        double T;

        /* Controller "memory" */
        double integrator;
        double prevError;			/* Required for integrator */
        double differentiator;
        double prevMeasurement;		/* Required for differentiator */

        public PIDController() {
            reset();
        }

        public PIDController(double Kp, double Ki, double Kd) {
            this();
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
        }

        public void init(double Kp, double Ki, double Kd) {
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
        }

        public void reset() {
            integrator = 0.0;
            prevError = 0.0;
            differentiator = 0.0;
            prevMeasurement = 0.0;
            T = 0.0;
        }

        public double update(double setpoint, double measurement, double time) {
            double output;
            T = time;

            /*
             * Error signal
             */
            double error = setpoint - measurement;


            /*
             * Proportional
             */
            double proportional = Kp * error;

            /*
             * Integral
             */
            integrator = integrator + 0.5f * Ki * T * (error + prevError);

            /* Anti-wind-up via integrator clamping */
            if (integrator > limMaxInt) {

                integrator = limMaxInt;

            } else if (integrator < limMinInt) {

                integrator = limMinInt;

            }


            /*
             * Compute output and apply limits
             */
            output = proportional + integrator;

            if (output > limMax) {

                output = limMax;

            } else if (output < limMin) {

                output = limMin;

            }

            /* Store error and measurement for later use */
            prevError       = error;
            prevMeasurement = measurement;

            /* Return controller output */
            return output;
        }
    }

    public double motionProfilePower(double max_acceleration, double max_velocity, double distance, double current_dt){

        double instantTargetPosition = motion_profile_position(max_acceleration,
                max_velocity,
                distance,
                current_dt);

        double motorPower = 0;
        return motorPower;
        //double motorPower = (instantTargetPosition - motor.getPosition()) * Kp;
    }

    double motion_profile_position(double max_acceleration, double max_velocity, double distance, double current_dt) {
        //"""
        //Return the current reference position based on the given motion profile times, maximum acceleration, velocity, and current time.
        //"""

        double acceleration_dt, halfway_distance, acceleration_distance,
                deacceleration_dt, cruise_distance, cruise_dt, deacceleration_time,
                entire_dt, cruise_current_dt;

        // calculate the time it takes to accelerate to max velocity
        acceleration_dt = max_velocity / max_acceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        halfway_distance = distance / 2;
        acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);

        if (acceleration_distance > halfway_distance)
            acceleration_dt = Math.sqrt(halfway_distance / (0.5 * max_acceleration));

        acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);

        // recalculate max velocity based on the time we have to accelerate and decelerate
        max_velocity = max_acceleration * acceleration_dt;

        // we decelerate at the same rate as we accelerate
        deacceleration_dt = acceleration_dt;

        // calculate the time that we're at max velocity
        cruise_distance = distance - 2 * acceleration_distance;
        cruise_dt = cruise_distance / max_velocity;
        deacceleration_time = acceleration_dt + cruise_dt;

        // check if we're still in the motion profile
        entire_dt = acceleration_dt + cruise_dt + deacceleration_dt;
        if (current_dt > entire_dt)
            return distance;

        // if we're accelerating
        if (current_dt < acceleration_dt) {
            // use the kinematic equation for acceleration
            return 0.5 * max_acceleration * Math.pow(current_dt, 2);
        }
        // if we're cruising
        else if (current_dt < deacceleration_time) {
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
            cruise_current_dt = current_dt - acceleration_dt;

            // use the kinematic equation for constant velocity
            return acceleration_distance + max_velocity * cruise_current_dt;
        }
        // if we're decelerating
        else {
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
            cruise_distance = max_velocity * cruise_dt;
            deacceleration_time = current_dt - deacceleration_time;

            // use the kinematic equations to calculate the instantaneous desired position
            return acceleration_distance + cruise_distance + max_velocity * deacceleration_time
                    - 0.5 * max_acceleration * Math.pow(deacceleration_time,2);
        }
    }
}
