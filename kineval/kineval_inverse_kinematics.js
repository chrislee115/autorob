
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
}

kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {
    // STENCIL: implement inverse kinematics iteration

    // [NOTICE]: Please assign the following 3 variables to test against CI grader

    // ---------------------------------------------------------------------------
    // robot.dx = []              // Error term, matrix size: 6 x 1, e.g., [[1],[1],[1],[0],[0],[0]]
    // robot.jacobian = []        // Jacobian matrix of current IK iteration matrix size: 6 x N
    // robot.dq = []              // Joint configuration change term (don't include step length)  
    // ---------------------------------------------------------------------------
    
    // TODO: verify this somewhere
    // Have to take the difference between the xyz and the rpy 
    var xd = endeffector_target_world;
    var xn = matrix_multiply(robot.joints[endeffector_joint].xform, endeffector_position_local);
    var curOrientation = robot.joints[endeffector_joint].origin.rpy;
    robot.dx = [0,0,0,0,0,0];
    for (var i = 0; i < 3; ++i) {
        robot.dx[i] = xd.position[i] - xn[i]
    }
    for (var i = 0; i < 3; ++i) {
        robot.dx[i + 3] = xd.orientation[i] - curOrientation[i];
    }

    var qn = robot.joints[endeffector_joint].angle;
    var jacobian; //TODO:
    if (kineval.params.ik_pseudoinverse) {
        invJ = matrix_pseudoinverse(jacobian) * matrix_copy(robot.dx);
    } else {
        invJ = matrix_multiply(matrix_transpose(jacobian), matrix_copy(robot.dx));
    }
    robot.jacobian = matrix_copy(matrix_multiply(invJ, robot.dx))
    
    // var gamma = kineval.params.ik_steplength
    // var gammaJ = matrix_copy(robot.jacobian)
    // gammaJ = gammaJ.map(function(x) { return x * gamma; });
    // robot.dq = qn + gammaJ

    // Explanation of above 3 variables:
    // robot.dq = T(robot.jacobian) * robot.dx  // where T(robot.jacobian) means apply some transformations to the Jacobian matrix, it could be Transpose, PseudoInverse, etc.
    // dtheta = alpha * robot.dq   // alpha: step length



}



