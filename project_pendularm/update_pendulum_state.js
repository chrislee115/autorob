first = true
function update_pendulum_state(numerical_integrator, pendulum, dt, gravity) {
    // integrate pendulum state forward in time by dt
    // please use names 'pendulum.angle', 'pendulum.angle_previous', etc. in else codeblock between line 28-30
    if (typeof numerical_integrator === "undefined")
        numerical_integrator = "none";

    if (numerical_integrator === "euler") {
        pendulum.angle_previous = pendulum.angle;
        prevV = pendulum.angle_dot
        pendulum.angle_dot = pendulum.angle_dot + (dt * pendulum_acceleration(pendulum, gravity));
        pendulum.angle = pendulum.angle + (dt * prevV);
        numerical_integrator = "euler";
    }
    else if (numerical_integrator === "verlet") {
        oldPrev = pendulum.angle
        pendulum.angle = (2 * pendulum.angle) - pendulum.angle_previous + (pendulum_acceleration(pendulum, gravity) * Math.pow(dt, 2))
        pendulum.angle_previous = oldPrev
        numerical_integrator = "verlet";
    }
    else if (numerical_integrator === "velocity verlet") {
        prevAccel = pendulum_acceleration(pendulum, gravity)
        
        oldPrev = pendulum.angle
        pendulum.angle = pendulum.angle + (pendulum.angle_dot * dt) + ((1/2) * prevAccel * Math.pow(dt, 2))
        pendulum.angle_previous = oldPrev
        pendulum.angle_dot = pendulum.angle_dot + ((prevAccel + pendulum_acceleration(pendulum, gravity)) / 2) * dt
        numerical_integrator = "velocity verlet";
    }
    else if (numerical_integrator === "runge-kutta") {

    // STENCIL: Runge-Kutta 4 integrator
    } 
    else {
        pendulum.angle_previous = pendulum.angle;
        pendulum.angle = (pendulum.angle+Math.PI/180)%(2*Math.PI);
        pendulum.angle_dot = (pendulum.angle-pendulum.angle_previous)/dt;
        numerical_integrator = "none";
    }

    return pendulum;
}

function pendulum_acceleration(pendulum, gravity) {
    // STENCIL: return acceleration(s) system equation(s) of motion 
    return  ((-1 * gravity * Math.sin(pendulum.angle)) / pendulum.length) + (pendulum.control / (pendulum.mass * Math.pow(pendulum.length,2)))
}

function init_verlet_integrator(pendulum, t, gravity) {
    // STENCIL: for verlet integration, a first step in time is needed
    // return: updated pendulum state and time
    oldPrev = pendulum.angle
    pendulum.angle = pendulum.angle + (pendulum.angle_dot * dt) + (Math.pow(dt, 2) * (1/2) * pendulum_acceleration(pendulum, gravity))
    pendulum.angle_previous = oldPrev
    t = t + dt
    return [pendulum, t];
}

function set_PID_parameters(pendulum) {
    // STENCIL: change pid parameters
    pendulum.servo = {kp:1000, kd:1000, ki:100};  // no control
    return pendulum;
}

function PID(pendulum, accumulated_error, dt) {
    // STENCIL: implement PID controller
    // return: updated output in pendulum.control and accumulated_error
    et = pendulum.desired - pendulum.angle
    etdt = pendulum.desired - pendulum.angle_previous
    // pendulum.angle_previous = pendulum.angle
    accumulated_error = accumulated_error + et
    pendulum.control = (pendulum.servo.kp * et) + (pendulum.servo.ki * accumulated_error) + (pendulum.servo.kd * (et - etdt))
    return [pendulum, accumulated_error];
}