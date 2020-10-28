
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
tmpIntStart = 0;
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
    // these parts of the robot should not be moving
    kineval.params.setpoint_target["shoulder_right_yaw"] = 0;
    kineval.params.setpoint_target["shoulder_left_yaw"] = 0;
    kineval.params.setpoint_target["upperarm_right_pitch"] = 0;
    kineval.params.setpoint_target["upperarm_left_pitch"] = 0;
    
    // FSM for arm movement, tries to match the movement to the song's timing
    if (curdate.getSeconds() - startTime >= 9)  {
        curTime = curdate.getSeconds() * 1000 + curdate.getMilliseconds();
        if (firstNine) { // sets the start time for this interval, to make timing interval calculation
            tmpIntStart = curTime;
            firstNine = false;
        }
        if (curTime - tmpIntStart < 200) { // STATE 2
            kineval.params.setpoint_target["clavicle_right_yaw"] = 2*Math.PI - Math.PI/8;
            kineval.params.setpoint_target["clavicle_left_roll"] = Math.PI/8;
        } 
        else if (curTime - tmpIntStart < 700) { // STATE 3
            kineval.params.setpoint_target["clavicle_right_yaw"] = 2*Math.PI - Math.PI/4;
            kineval.params.setpoint_target["clavicle_left_roll"] = Math.PI/4;
        }
        else if (curTime - tmpIntStart < 900) { // STATE 2
            kineval.params.setpoint_target["clavicle_right_yaw"] = 2*Math.PI - Math.PI/8;
            kineval.params.setpoint_target["clavicle_left_roll"] = Math.PI/8;
        } 
        else if (curTime - tmpIntStart < 1200) { // STATE 3
            kineval.params.setpoint_target["clavicle_right_yaw"] = 2*Math.PI - Math.PI/4;
            kineval.params.setpoint_target["clavicle_left_roll"] = Math.PI/4;
        } else {
            // full reset of the FSM
            // AKA back to first state
            startTime = curdate.getSeconds();
            firstNine = true;
            firstSeven = true;
            firstFive = true;
        }
    }
    else if (curdate.getSeconds() - startTime >= 7)  {
        if (firstSeven) {
            if (curdate.getMilliseconds() >= 500) { // STATE 3
                kineval.params.setpoint_target["clavicle_right_yaw"] = 2*Math.PI - Math.PI/4;
                kineval.params.setpoint_target["clavicle_left_roll"] = Math.PI/4;
                firstSeven = false;
            }
        } 
    }
    else if (curdate.getSeconds() - startTime >= 5) {
        if (firstFive) {
            if (curdate.getMilliseconds() >= 200) { // STATE 2
                kineval.params.setpoint_target["clavicle_right_yaw"] = 2*Math.PI - Math.PI/8;
                kineval.params.setpoint_target["clavicle_left_roll"] = Math.PI/8;
                firstFive = false;
            }
        } 
    } else { // STATE 1
        kineval.params.setpoint_target["clavicle_right_yaw"] = -1 * (2*Math.PI - Math.PI/8);
        kineval.params.setpoint_target["clavicle_left_roll"] = -1 * Math.PI/8;
    }

    // FSM for "taps", timed to song
    // up and down movements in the forearms
    if (curdate.getMilliseconds() % 270 >= 135) {
        kineval.params.setpoint_target["forearm_right_yaw"] = 0;
        kineval.params.setpoint_target["forearm_left_yaw"] = 0;
    } else {
        kineval.params.setpoint_target["forearm_right_yaw"] = Math.PI/6;
        kineval.params.setpoint_target["forearm_left_yaw"] = Math.PI/6;
    }
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


