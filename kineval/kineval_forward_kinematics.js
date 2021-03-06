
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



    // lateral_trans = generate_translation_matrix(1, 0, 0);
    // robot_lateral = matrix_multiply(lateral_trans, tempXYZ);

    // initialize base=
    temp = matrix_copy(generate_identity());  
    temp = matrix_multiply(temp, generate_translation_matrix(robot.origin.xyz[0], robot.origin.xyz[1], robot.origin.xyz[2]));
    temp = matrix_multiply(temp,  generate_rotation_matrix_Z(robot.origin.rpy[2]));
    temp = matrix_multiply(temp,  generate_rotation_matrix_Y(robot.origin.rpy[1]));
    temp = matrix_multiply(temp, generate_rotation_matrix_X(robot.origin.rpy[0]));

    robot.origin.xform = matrix_copy(temp);
    
    headTemp = matrix_copy(temp);
    robot_heading = matrix_multiply(headTemp, generate_translation_matrix(0,0,1));
    robot_heading = [[robot_heading[0][3]],
    [robot_heading[1][3]],
    [robot_heading[2][3]],
    [robot_heading[3][3]]];

    laTemp = matrix_copy(temp);
    robot_lateral = matrix_multiply(laTemp, generate_translation_matrix(1,0,0));
    robot_lateral = [[robot_lateral[0][3]],
    [robot_lateral[1][3]],
    [robot_lateral[2][3]],
    [robot_lateral[3][3]]];

    traverseFKBase();
}
function traverseFKBase() {
    baseName = robot.base;
    robot.links[baseName].xform = matrix_copy(robot.origin.xform);
    children = robot.links[baseName].children;

    children.forEach(function(child_joint) {
        traverseFKJoint(child_joint);
    });
}
function traverseFKLink(link_in) {
    robot.links[link_in].xform = matrix_copy(robot.joints[robot.links[link_in].parent].xform);

    // end traversal if no children, 
    // otherwise recurse to joint
    children = robot.links[link_in].children; 
    if (children != undefined) {
        children.forEach(function(child_joint) {
            traverseFKJoint(child_joint);
        });
    } 
}
function traverseFKJoint(joint_in) {
    tempMstack = matrix_copy(robot.links[robot.joints[joint_in].parent].xform);
    tempRPY = robot.joints[joint_in].origin.rpy;
    tempXYZ = robot.joints[joint_in].origin.xyz;
    tempMstack = matrix_multiply(tempMstack, generate_translation_matrix(tempXYZ[0], tempXYZ[1], tempXYZ[2]));
    tempMstack = matrix_multiply(tempMstack,  generate_rotation_matrix_Z(tempRPY[2]));
    tempMstack = matrix_multiply(tempMstack,  generate_rotation_matrix_Y(tempRPY[1]));
    tempMstack = matrix_multiply(tempMstack, generate_rotation_matrix_X(tempRPY[0]));

    // assignment 4 rotation
    tmpAxis = robot.joints[joint_in].axis;
    tmpAngle = robot.joints[joint_in].angle;
    tmpQuat = kineval.quaternionFromAxisAngle(tmpAxis, tmpAngle);
    quatRot = kineval.quaternionToRotationMatrix(tmpQuat);
    tempMstack = matrix_multiply(tempMstack, quatRot);
    robot.joints[joint_in].xform = matrix_copy(tempMstack);

    if (robot.joints[joint_in].child != undefined) {
        traverseFKLink(robot.joints[joint_in].child);
    }
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

