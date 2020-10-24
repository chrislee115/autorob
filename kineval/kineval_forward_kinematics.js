
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotForwardKinematics = function robotForwardKinematics () { 
    if (typeof kineval.buildFKTransforms === 'undefined') {
        textbar.innerHTML = "forward kinematics not implemented";
        return;
    }
    kineval.buildFKTransforms();
    // STENCIL: implement kineval.buildFKTransforms();

}

kineval.buildFKTransforms = function buildFKTransforms() {
    console.log("we out here boiiiii\n");
    //push a copy of top of stack when traversing to child
    if (robot.origin.xform == undefined) {
        robot.origin.xform = generate_identity();
    }
    tempmat = matrix_copy(robot.origin.xform);
    tempmat  = matrix_multiply(tempmat, generate_translation_matrix(robot.origin.xyz[0], robot.origin.xyz[1], robot.origin.xyz[2]));
    tempmat  = matrix_multiply(tempmat, generate_rotation_matrix_X(robot.origin.rpy[0]));
    tempmat = matrix_multiply(tempmat,  generate_rotation_matrix_Y(robot.origin.rpy[1]));
    tempmat = matrix_multiply(tempmat,  generate_rotation_matrix_Z(robot.origin.rpy[2]));
    robot.origin.xform = tempmat;
}

function traverseFKBase() {
    console.log("hoya\n");
}
function traverseFKLink() {
    console.log("hoya\n");
}
function traverseFKJoint() {
    console.log("hoya\n");
}
    // STENCIL: reference code alternates recursive traversal over 
    //   links and joints starting from base, using following functions: 
    //     traverseFKBase
    //     traverseFKLink
    //     traverseFKJoint
    //
    // user interface needs the heading (z-axis) and lateral (x-axis) directions
    //   of robot base in world coordinates stored as 4x1 matrices in
    //   global variables "robot_heading" and "robot_lateral"
    //
    // if geometries are imported and using ROS coordinates (e.g., fetch),
    //   coordinate conversion is needed for kineval/threejs coordinates:
    //

