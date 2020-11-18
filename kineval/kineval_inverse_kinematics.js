
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | inverse kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotInverseKinematics = function robot_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // compute joint angle controls to move location on specified link to Cartesian location
    if ((kineval.params.update_ik)||(kineval.params.persist_ik)) { 
        // if update requested, call ik iterator and show endeffector and target
        kineval.iterateIK(endeffector_target_world, endeffector_joint, endeffector_position_local);
        if (kineval.params.trial_ik_random.execute)
            kineval.randomizeIKtrial();
        else // KE: this use of start time assumes IK is invoked before trial
            kineval.params.trial_ik_random.start = new Date();
    }

    kineval.params.update_ik = false; // clear IK request for next iteration
}

kineval.randomizeIKtrial = function randomIKtrial () {

    // update time from start of trial
    cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime()-kineval.params.trial_ik_random.start.getTime();

    // STENCIL: see instructor for random time trial code
    endeffector_world = matrix_multiply(robot.joints[robot.endeffector.frame].xform,robot.endeffector.position);

    // compute distance of endeffector to target
    kineval.params.trial_ik_random.distance_current = Math.sqrt(
            Math.pow(kineval.params.ik_target.position[0][0]-endeffector_world[0][0],2.0)
            + Math.pow(kineval.params.ik_target.position[1][0]-endeffector_world[1][0],2.0)
            + Math.pow(kineval.params.ik_target.position[2][0]-endeffector_world[2][0],2.0) );

    // if target reached, increment scoring and generate new target location
    // KE 2 : convert hardcoded constants into proper parameters
    if (kineval.params.trial_ik_random.distance_current < 0.01) {
        kineval.params.ik_target.position[0][0] = 1.2*(Math.random()-0.5);
        kineval.params.ik_target.position[1][0] = 1.2*(Math.random()-0.5)+1.5;
        kineval.params.ik_target.position[2][0] = 0.7*(Math.random()-0.5)+0.5;
        kineval.params.trial_ik_random.targets += 1;
        textbar.innerHTML = "IK trial Random: target " + kineval.params.trial_ik_random.targets + " reached at time " + kineval.params.trial_ik_random.time;
    }
}


kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {
    // STENCIL: implement inverse kinematics iteration

    // [NOTICE]: Please assign the following 3 variables to test against CI grader

    // ---------------------------------------------------------------------------
    // robot.dx = []              // Error term, matrix size: 6 x 1, e.g., [[1],[1],[1],[0],[0],[0]]
    // robot.jacobian = []        // Jacobian matrix of current IK iteration matrix size: 6 x N
    // robot.dq = []              // Joint configuration change term (don't include step length)  
    // ---------------------------------------------------------------------------
    // Get all joints on endeffector path

    var joints = [endeffector_joint];
    var curJoint = robot.joints[endeffector_joint];
    while (true) {
        var parentLink = curJoint.parent;
        if (parentLink == 'base') {
            break;
        }
        joints.unshift(robot.links[parentLink].parent);
        curJoint = robot.joints[robot.links[parentLink].parent];
    }
    // calculates the jacobian
    robot.jacobian = [[],[],[],[],[],[]];
    joints.forEach(function(cur_joint) {
        cur_joint = robot.joints[cur_joint];
    
        var tmpAxis = [[0],[0],[0],[1]];
        for (var i = 0; i < 3; ++i) {
            tmpAxis[i][0] = cur_joint.axis[i];
        }
        var tiw = matrix_copy(cur_joint.xform);
        var kiw = matrix_multiply(matrix_copy(tiw), tmpAxis);

        tiw = matrix_copy(cur_joint.xform);
        var zero = [[0],[0],[0],[1]];
        var oiw = matrix_multiply(matrix_copy(tiw), zero);
        
        tiw = matrix_copy(robot.joints[endeffector_joint].xform);
        var ptool = matrix_multiply(tiw, endeffector_position_local);
        
        var kiwMinusOiw = [];
        for (var i = 0; i < oiw.length; ++i) {
            kiwMinusOiw.push(kiw[i][0] - oiw[i][0]);
        }
        var ptoolMinusOiw = [];
        for (var i = 0; i < oiw.length; ++i) {
            ptoolMinusOiw.push(ptool[i][0] - oiw[i][0]);
        }
        var cross = vector_cross(kiwMinusOiw, ptoolMinusOiw);
        for (var i = 0; i < 3; ++i) {
            // Jvi - linear
            robot.jacobian[i].push(cross[i]);
            // Jwi - Angular
            robot.jacobian[i + 3].push(kiwMinusOiw[i]);
        }
    }); 
    var cur_joint = robot.joints[endeffector_joint];
    // Have to take the difference between the xyz and the rpy 
    var xd = endeffector_target_world;
    var curPosition = matrix_multiply(matrix_copy(cur_joint.xform), endeffector_position_local);
    // am not sure if this is remotely right
    var curOrientation = cur_joint.origin.rpy;
    robot.dx = [[0],[0],[0],[0],[0],[0]];
    for (var i = 0; i < 3; ++i) {
        robot.dx[i][0] = xd.position[i] - curPosition[i];
    }
    // OFFICE HOURS - this is supopsed to be kept at 0?
    // for (var i = 0; i < 3; ++i) {
    //     robot.dx[i + 3][0] = xd.orientation[i] - curOrientation[2 - i];
    // }

    var jacobian = matrix_copy(robot.jacobian);
    if (kineval.params.ik_pseudoinverse) {
        robot.dq = matrix_multiply(matrix_pseudoinverse(jacobian), matrix_copy(robot.dx));
    } else {
        robot.dq = matrix_multiply(matrix_transpose(jacobian), matrix_copy(robot.dx));
    }
    var gamma = kineval.params.ik_steplength;
    var i = 0;
    joints.forEach(function(cur_joint) {
        cur_joint = robot.joints[cur_joint];
        cur_joint.angle = cur_joint.angle + (gamma * robot.dq[i][0]);
        i = i + 1;
    });
    // Explanation of above 3 variables:
    // robot.dq = T(robot.jacobian) * robot.dx  // where T(robot.jacobian) means apply some transformations to the Jacobian matrix, it could be Transpose, PseudoInverse, etc.
    // dtheta = alpha * robot.dq   // alpha: step length



}



