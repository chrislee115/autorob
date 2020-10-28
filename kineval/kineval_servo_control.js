
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | arm servo control

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/
startTime = 0;
newStartTime = 0;
kineval.setpointDanceSequence = function execute_setpoints() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_dance) {
        song.pause();
        return;
    }

    var curdate = new Date();
    if (song.paused) {
        startTime = curdate.getSeconds();
        song.load();
        song.play();   
        firstNine = true;
        firstSeven = true;
        firstFive = true;
    }
    // if (curdate.getSeconds() - startTime >= 10)  {
    //     if (curdate.getMilliseconds() < 100) {
    //         kineval.params.setpoint_target["clavicle_right_yaw"] = 2*Math.PI - Math.PI/8;
    //         kineval.params.setpoint_target["clavicle_left_roll"] = Math.PI/8;
    //     } else {
    //         kineval.params.setpoint_target["clavicle_right_yaw"] = 2*Math.PI - Math.PI/4;
    //         kineval.params.setpoint_target["clavicle_left_roll"] = Math.PI/4;
    //     }
    // }
    kineval.params.setpoint_target["shoulder_right_yaw"] = 0;
    kineval.params.setpoint_target["shoulder_left_yaw"] = 0;
    kineval.params.setpoint_target["upperarm_right_pitch"] = 0;
    kineval.params.setpoint_target["shoulder_left_yaw"] = 0;
    if (curdate.getSeconds() - startTime >= 9)  {
        curTime = curdate.getSeconds() * 1000 + curdate.getMilliseconds();
        if (firstNine) {
            newStartTime = curTime;
            firstNine = false;
        }
        if (curTime - newStartTime < 200) {
            kineval.params.setpoint_target["clavicle_right_yaw"] = 2*Math.PI - Math.PI/8;
            kineval.params.setpoint_target["clavicle_left_roll"] = Math.PI/8;
        } 
        else if (curTime - newStartTime < 700) {
            kineval.params.setpoint_target["clavicle_right_yaw"] = 2*Math.PI - Math.PI/4;
            kineval.params.setpoint_target["clavicle_left_roll"] = Math.PI/4;
        }
        else if (curTime - newStartTime < 900) {
            kineval.params.setpoint_target["clavicle_right_yaw"] = 2*Math.PI - Math.PI/8;
            kineval.params.setpoint_target["clavicle_left_roll"] = Math.PI/8;
        } 
        else if (curTime - newStartTime < 1200) {
            kineval.params.setpoint_target["clavicle_right_yaw"] = 2*Math.PI - Math.PI/4;
            kineval.params.setpoint_target["clavicle_left_roll"] = Math.PI/4;
        } else {
            startTime = curdate.getSeconds();
            firstNine = true;
            firstSeven = true;
            firstFive = true;
        }
    }
    else if (curdate.getSeconds() - startTime >= 7)  {
        if (firstSeven) {
            if (curdate.getMilliseconds() >= 500) {
                kineval.params.setpoint_target["clavicle_right_yaw"] = 2*Math.PI - Math.PI/4;
                kineval.params.setpoint_target["clavicle_left_roll"] = Math.PI/4;
                firstSeven = false;
            }
        } else {
            kineval.params.setpoint_target["clavicle_right_yaw"] = 2*Math.PI - Math.PI/4;
            kineval.params.setpoint_target["clavicle_left_roll"] = Math.PI/4;
        }
    }
    else if (curdate.getSeconds() - startTime >= 5) {
        if (firstFive) {
            if (curdate.getMilliseconds() >= 200) {
                kineval.params.setpoint_target["clavicle_right_yaw"] = 2*Math.PI - Math.PI/8;
                kineval.params.setpoint_target["clavicle_left_roll"] = Math.PI/8;
                firstFive = false;
            }
        } else {
            kineval.params.setpoint_target["clavicle_right_yaw"] = 2*Math.PI - Math.PI/8;
            kineval.params.setpoint_target["clavicle_left_roll"] = Math.PI/8;
        }
    } else {
        kineval.params.setpoint_target["clavicle_right_yaw"] = -1 * (2*Math.PI - Math.PI/8);
        kineval.params.setpoint_target["clavicle_left_roll"] = -1 * Math.PI/8;
    }

    //up and down movements
    if (curdate.getMilliseconds() % 270 >= 135) {
        kineval.params.setpoint_target["forearm_right_yaw"] = 0;
        kineval.params.setpoint_target["forearm_left_yaw"] = 0;
    } else {
        kineval.params.setpoint_target["forearm_right_yaw"] = Math.PI/6;
        kineval.params.setpoint_target["forearm_left_yaw"] = Math.PI/6;
    }
    // STENCIL: implement FSM to cycle through dance pose setpoints
}

kineval.setpointClockMovement = function execute_clock() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_clock) return; 
    var curdate = new Date();
    for (x in robot.joints) {
        kineval.params.setpoint_target[x] = curdate.getSeconds()/60*2*Math.PI;
    }
}


kineval.robotArmControllerSetpoint = function robot_pd_control () {

    // if update not requested, exit routine
    if ((!kineval.params.update_pd)&&(!kineval.params.persist_pd)) return; 

    kineval.params.update_pd = false; // if update requested, clear request and process setpoint control

    // STENCIL: implement P servo controller over joints
    for (x in robot.joints) {
        et = kineval.params.setpoint_target[x] - robot.joints[x].angle;
        robot.joints[x].control = .5 * et;
    }
    // et = pendulum.desired - pendulum.angle
    // pendulum.control = (pendulum.servo.kp * et)
}


