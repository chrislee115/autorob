/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    2D Path Planning in HTML5 Canvas | RRT Methods

    Stencil methods for student implementation of RRT-based search.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

    Usage: see search_canvas.html

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

function iterateRRT() {


    // STENCIL: implement a single iteration of an RRT algorithm.
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "extended" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   insertTreeVertex - adds and displays new configuration vertex for a tree
    //   insertTreeEdge - adds and displays new tree edge between configurations
    //   drawHighlightedPath - renders a highlighted path in a tree
}
function getDistToQRand(q1, q2) {
    return Math.pow(q1[0] - q2[0], 2) + 
                            Math.pow(q1[1] - q2[1], 2);
}
function vector_normalize(v) {
    // returns normalized vector for v
    var magnitude = 0
    for (var i = 0; i < v.length; ++i) {
        magnitude += (Math.pow(v[i], 2));
    }
    magnitude = Math.sqrt(magnitude);
    for (var i = 0; i < v.length; ++i) {
        v[i] = v[i] / magnitude;
    }
    return v;
}
// double check legality lol
function getQNew(q, q_near) {
    var q_new = [ (q[0] - q_near[0]), (q[1] - q_near[1]) ];
    q_new = vector_normalize(q_new);
    q_new = [ q_near[0] + (q_new[0] * eps), 
              q_near[1] + (q_new[1] * eps) ]
    return q_new;
}

function isEqual(q1, q2) {
    return (Math.abs(q1[0] - q2[0]) < eps) && 
            (Math.abs(q1[1] - q2[1]) < eps);
}
// q is q_rand
function extendRRT(T, q) {
    var q_near_tmp = findNearestNeighbor(T, q);
    var q_near = q_near_tmp[0];
    var q_near_idx = q_near_tmp[1];
    var q_new = getQNew(q, q_near);
    
    // I don't see why a new config function is needed
    if (!testCollision(q_new)) {
        insertTreeVertex(T, q_new);
        var q_new_idx = T.newest;
        insertTreeEdge(T, q_near_idx, q_new_idx);

        // Double check this equality
        if (isEqual(q_new, q)) {
            return "Reached";
        } else {
            return "Advanced";
        }
    }
    return "Trapped";
}

function connectRRT(T, q) {
    do {
        S = extendRRT(T, q);
    } while (S == "Advanced");
    return S;
}

function randomConfig() {
    var x = (Math.random() * 10) - 2;
    var y = (Math.random() * 10) - 2;
    return [x,y];
}


function findNearestNeighbor(T, q) {
    var best = 10000000000;
    var best_index = -1;
    for (var i = 0; i < T.vertices.length; ++i) {
        // This is how we do dist to goal right
        var cur_dist = getDistToQRand(T.vertices[i].vertex, q);
        if (cur_dist < best) {
            best = cur_dist;
            best_index = i;
        }
    }
    return [T.vertices[best_index].vertex, best_index];
}

function findVertexIdx(T, vtx) {
    for (var i = 0; i < T.vertices.length; ++i) {
        if (T.vertices[i].vertex == vtx) {
            return i;
        }    
    }
    return -1;
}
function ihatejs(arr, vtx) {
    for (var i = 0; i < arr.length; ++i) {
        if (arr[i].vertex == vtx) {
            return i;
        }
    }
    return -1;
}
function dfsPath(path, T, q) {
    //TODO: for reconstructing path when were done
    var isInit = (T.vertices[0].vertex == q_init);
    var goal = isInit ? q : q_goal;
    var dfs = [];
    var visited = [];
    var start = isInit ? q_init : q;
    if (findVertexIdx(T, start) == -1) {
        var tmp = findNearestNeighbor(T, start);
        start = tmp[0];
    }
    console.log(findVertexIdx(T, start))
    dfs.push(start);
    while (dfs.length != 0) {
        var cur_vtx = dfs.pop();
        var cur_vtx_idx = findVertexIdx(T, cur_vtx);

        var tmp_obj = {}
        tmp_obj.vertex = cur_vtx;

        visited.push(tmp_obj);

        var parent_idx = ihatejs(visited, cur_vtx);
        if (isEqual(cur_vtx, goal)) {
            var cur_node = visited[parent_idx];
            while (true) {
                var tmp_obj2 = {};
                tmp_obj2.vertex = cur_node.vertex;
                if (path.indexOf(tmp_obj2) == -1) {
                    path.unshift(tmp_obj2);
                }
                if (cur_node.parent == undefined) break;
                cur_node = visited[cur_node.parent];
            }
            return path;
        }

        var neighbors = T.vertices[cur_vtx_idx].edges;
        for (var i = 0; i < neighbors.length; ++i) {
            var neighbor = neighbors[i].vertex;
            if (ihatejs(visited, neighbor) == -1) {
                dfs.push(neighbor);

                var tmp_obj2 = {}
                tmp_obj2.vertex = neighbor;
                visited.push(tmp_obj2);
                visited[visited.length-1].parent = parent_idx;
            }
        }
    }
    // should not get here
    return dfs;
}
function iterateRRTConnect() {


    // STENCIL: implement a single iteration of an RRT-Connect algorithm.
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "extended" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   insertTreeVertex - adds and displays new configuration vertex for a tree
    //   insertTreeEdge - adds and displays new tree edge between configurations
    //   drawHighlightedPath - renders a highlighted path in a tree

    // // search_max_iterations == K?
    if (search_iter_count > search_max_iterations) {
        search_iterate = false;
        return "failed";
    }
    var q_rand = randomConfig();
    if (extendRRT(T_a, q_rand) != "Trapped") {
        var q_new = T_a.vertices[T_a.newest].vertex;
        if (connectRRT(T_b, q_new) == "Reached") {
            // idk if this works
            var path = [];
            path = dfsPath(path, T_a, q_new);
            path = dfsPath(path, T_b, q_new);
            drawHighlightedPath(path);
            search_iterate = false;
            return "succeeded";
        } 
    }
    // swap
    [T_a, T_b] = [T_b, T_a];
    return "extended";
}

function iterateRRTStar() {

}

//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

    // STENCIL: implement RRT-Connect functions here, such as:
    //   extendRRT
    //   connectRRT
    //   randomConfig
    //   newConfig
    //   findNearestNeighbor
    //   dfsPath
