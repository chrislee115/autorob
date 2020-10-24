//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

// STENCIL: reference quaternion code has the following functions:
//   quaternion_from_axisangle
//   quaternion_normalize
//   quaternion_to_rotation_matrix
//   quaternion_multiply

// **** Function stencils are provided below, please uncomment and implement them ****//

kineval.quaternionFromAxisAngle = function quaternion_from_axisangle(axis,angle) {
    // returns quaternion q as dic, with q.a as real number, q.b as i component, q.c as j component, q.d as k component
    var q = {};
    q.a = Math.cos(angle/2);
    q.b = axis[0] * Math.sin(angle/2);
    q.c = axis[1] * Math.sin(angle/2);
    q.d = axis[2] * Math.sin(angle/2);
    return q;
}

kineval.quaternionNormalize = function quaternion_normalize(q1) {
    // returns quaternion q as dic, with q.a as real number, q.b as i component, q.c as j component, q.d as k component
    var q = {};
    norm = Math.sqrt(Math.pow(q1.a,2), Math.pow(q1.b,2), Math.pow(q1.c,2), Math.pow(q1.d,2));
    q.a = q1.a / norm;
    q.b = q1.b / norm;
    q.c = q1.c / norm;
    q.d = q1.d / norm;
    return q;
}

kineval.quaternionMultiply = function quaternion_multiply(q1,q2) {
    // returns quaternion q as dic, with q.a as real number, q.b as i component, q.c as j component, q.d as k component
    var q = {};
    q.a = ((q1.a * q2.a) - (q1.b * q2.b) - (q1.c * q2.c) - (q1.d * q2.d))
    q.b = ((q1.a * q2.b) + (q1.b * q2.a) + (q1.c * q2.d) - (q1.d * q2.c))
    q.c = ((q1.a * q2.c) - (q1.b * q2.d) + (q1.c * q2.a) + (q1.d * q2.b))
    q.d = ((q1.a * q2.d) + (q1.b * q2.c) - (q1.c * q2.b) + (q1.d * q2.a))
    return q;
}

kineval.quaternionToRotationMatrix = function quaternion_to_rotation_matrix (q) {    
    q1 = [
        [q.a, q.d, -1 * q.c, q.b],
        [-1 * q.d, q.a, q.b, q.c],
        [q.c, -1 * q.b, q.a, q.d], 
        [-1 * q.b, -1 * q.c, -1 * q.d, q.a]
    ]
    q2 = [
        [q.a, q.d, -1 * q.c, -1 * q.b],
        [-1 * q.d, q.a, q.b, -1 * q.c],
        [q.c, -1 * q.b, q.a, -1 * q.d], 
        [q.b, q.c, q.d, q.a]
    ]
    return matrix_multiply(q1, q2);
}