//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

function matrix_copy(m1) {
    // returns 2D array that is a copy of m1

    var mat = [];
    var i,j;

    for (i=0;i<m1.length;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<m1[0].length;j++) { // for each column of m1
            mat[i][j] = m1[i][j];
        }
    }
    return mat;
}


// STENCIL: reference matrix code has the following functions:
//   matrix_multiply
//   matrix_transpose
//   matrix_pseudoinverse
//   matrix_invert_affine
//   vector_normalize
//   vector_cross
//   generate_identity
//   generate_translation_matrix
//   generate_rotation_matrix_X
//   generate_rotation_matrix_Y
//   generate_rotation_matrix_Z



// **** Function stencils are provided below, please uncomment and implement them ****//



//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

function matrix_copy(m1) {
    // returns 2D array that is a copy of m1

    var mat = [];
    var i,j;

    for (i=0;i<m1.length;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<m1[0].length;j++) { // for each column of m1
            mat[i][j] = m1[i][j];
        }
    }
    return mat;
}


    // STENCIL: reference matrix code has the following functions:
    //   matrix_multiply
    //   matrix_transpose
    //   matrix_pseudoinverse
    //   matrix_invert_affine
    //   vector_normalize
    //   vector_cross
    //   generate_identity
    //   generate_translation_matrix
    //   generate_rotation_matrix_X
    //   generate_rotation_matrix_Y
    //   generate_rotation_matrix_Z

function matrix_multiply(m1, m2) {
    var m1r = m1.length, m1c = m1[0].length, m2r = m2.length, m2c = m2[0].length;
    // new matrix has rows of m1 and col of m2
    var ans = new Array(m1r);
    for (var r = 0; r < m1r; ++r) {
        ans[r] = new Array(m2c);
        for (var c = 0; c < m2c; ++c) {
            ans[r][c] = 0;
            // get dot product
            for (var mid = 0; mid < m2c; ++mid) {
                ans[r][mid] += (m1[r][i] * m2[i][c])
            }
        }
    }
    return ans;
}

// this might be wrong lol
function matrix_transpose(m) {
    m[0].map((_, col) => m.map(row => row[col]));
}

// function matrix_pseudoinverse(m) {
//     // returns pseudoinverse of matrix m

// }

// function matrix_invert_affine(m) {
//     // returns 2D array that is the invert affine of 4-by-4 matrix m

// }

function vector_normalize(v) {
    // returns normalized vector for v
    magnitude = 0
    for (var i = 0; i < v.length; ++i) {
        magnitude += (Math.pow(v[i], 2));
    }
    magnitude = Math.sqrt(magnitude);
    for (var i = 0; i < v.length; ++i) {
        v[i] = v[i] / magnitude;
    }
    return v;
}

function vector_cross(a,b) {
    ans = new Array(3);
    // return cross product of vector a and b with both has 3 dimensions
    ans[0] = (a[1]*b[2]) - (a[2]*b[1]);
    ans[1] = (a[2]*b[0]) - (a[0]*b[2]);
    ans[2] = (a[0]*b[1]) - (a[1]*b[0]);
    return ans;
}

function generate_identity() {
    // returns 4-by-4 2D array of identity matrix
    ans = new Array(4);
    for (var i = 0; i < 4; ++i) {
        ans[i] = new Array(4);
        for (var j = 0; j < 4; ++j) {
            ans[i][i] = 1;
        }
    }
    return ans;
}

function generate_translation_matrix(tx, ty, tz) {
    // returns 4-by-4 matrix as a 2D array
    ans = generate_identity();
    ans[0][3] = tx;
    ans[1][3] = ty;
    ans[2][3] = tz;
    return ans;
}

// function generate_rotation_matrix_X(angle) {
//     // returns 4-by-4 matrix as a 2D array, angle is in radians
    
// }

// function generate_rotation_matrix_Y(angle) {
//     // returns 4-by-4 matrix as a 2D array, angle is in radians
    
// }

// function generate_rotation_matrix_Z(angle) {
//     // returns 4-by-4 matrix as a 2D array, angle is in radians
    
// }