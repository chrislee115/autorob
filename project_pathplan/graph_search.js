/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    2D Path Planning in HTML5 Canvas | Graph Search Methods

    Stencil methods for student implementation of graph search algorithms.

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
var roundFactor = 0;
var roundedGoal = [0,0];
var currPath = [];
function updateRoundFactor() {
    roundFactor = 0;
    var tmp = eps;
    while (tmp < 1) {
        ++roundFactor;
        tmp = tmp * 10;
    }
    console.log(roundFactor);
    roundedGoal[0] = Number(q_goal[0].toFixed(roundFactor));
    roundedGoal[1] = Number(q_goal[1].toFixed(roundFactor));
    console.log(roundedGoal);
}
function initSearchGraph() {
    updateRoundFactor();
    // create the search queue
    visit_queue = [];
    // initialize search graph as 2D array over configuration space
    //   of 2D locations with specified spatial resolution
    G = [];
    for (iind=0,xpos=-2;xpos<7;iind++,xpos+=eps) {
        G[iind] = [];
        for (jind=0,ypos=-2;ypos<7;jind++,ypos+=eps) {
            G[iind][jind] = {
                i:iind,j:jind, // mapping to graph array
                x:Number(xpos.toFixed(roundFactor)),
                y:Number(ypos.toFixed(roundFactor)), // mapping to map coordinates
                parent:null, // pointer to parent in graph along motion path
                //gonna assume distance is equal to g
                distance:10000, // distance to start via path through parent
                visited:false, // flag for whether the node has been visited
                //gonna assume prio is fscore (g + h)
                priority:null, // visit priority based on fscore
                //this is "closed" list i think
                queued:false // flag for whether the node has been queued for visiting
            };
            // STENCIL: determine whether this graph node should be the start
            //   point for the search
            if ( (Math.abs(G[iind][jind].x - q_init[0]) < eps) && 
                 (Math.abs(G[iind][jind].y - q_init[1]) < eps)) {
                G[iind][jind].distance = 0;
                G[iind][jind].visited = true;
                G[iind][jind].queued = true;
                G[iind][jind].priority = getDistToGoal(G[iind][jind]);
                insert(visit_queue, G[iind][jind]);
            }

        }
    }
}
function getDistToPoint(elem1, elem2) {
    return Math.pow(elem1.x - elem2.x, 2) + 
                            Math.pow(elem1.y - elem2.y, 2);
}
function getDistToGoal(elem) {
    return Math.pow(elem.x - roundedGoal[0], 2) + 
                            Math.pow(elem.y - roundedGoal[1], 2);
}
function addNeighbors(curr, neighbors) {
    for (var i = 0; i < 3; ++i) {
        var j;
        j = (i % 2 == 0) ? 1 : 0;
        for (; j < 3; j += 2) {
            if ( (-2 <= G[curr.i - 1 + i][curr.j - 1 + j].x && 
                G[curr.i - 1 + i][curr.j - 1 + j].x < 7) &&
                (-2 <= G[curr.i - 1 + i][curr.j - 1 + j].y && 
                    G[curr.i - 1 + i][curr.j - 1 + j].y < 7)) {
                if (G[curr.i - 1 + i][curr.j - 1 + j].visited) {
                    continue;
                }
                if (testCollision([G[curr.i - 1 + i][curr.j - 1 + j].x, 
                    G[curr.i - 1 + i][curr.j - 1 + j].y])) { 
                    continue;
                }
                neighbors.push(G[curr.i - 1 + i][curr.j - 1 + j])
            }
        }
    }
}
function isGoal(pt) {
    return ((pt.x == roundedGoal[0]) && 
    (pt.y == roundedGoal[1]))
}
function iterateGraphSearch() {
    //probs not necessary
    if (search_visited == 0) { search_visited = 1; }
    if (visit_queue.length == 0) {
        search_iterate = false;
        return "failed";
    }
    if (!isGoal(visit_queue[0])) {
    // if (getDistToGoal(visit_queue[0]) >= (eps / 10.0)) {
        var curr = pop(visit_queue);
        draw_2D_configuration([curr.x, curr.y], "visited");
        curr.visited = true;
        ++search_visited;
        neighbors = [];
        //collision detection in addneighbors
        addNeighbors(curr, neighbors);
        for (var i = 0; i < neighbors.length; ++i) {
            //this is g
            var tempDist = curr.distance + eps;
            if (neighbors[i].distance > tempDist) {
                currPath.push(neighbors[i]);
                neighbors[i].distance = tempDist;
                neighbors[i].parent = curr;
                // g + h ( we dont store h )
                neighbors[i].priority = tempDist + getDistToGoal(neighbors[i]);
            }
            if (!neighbors[i].queued) {
                neighbors[i].queued = true;
                neighbors[i].parent = curr;
                insert(visit_queue, neighbors[i]);
            } 
            draw_2D_configuration([neighbors[i].x, neighbors[i].y], "queued");
        }
        return "iterating";
    } else {
        drawHighlightedPathGraph(visit_queue[0]);
        console.log(visit_queue[0])
        console.log(roundedGoal);
        search_iterate = false;
        return "succeeded";
    }

    // STENCIL: implement a single iteration of a graph search algorithm
    //   for A-star (or DFS, BFS, Greedy Best-First)
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "iterating" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   drawHighlightedPathGraph - draws a path back to the start location
    //   draw_2D_configuration - draws a square at a given location
}

//////////////////////////////////////////////////
/////     MIN HEAP IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

// STENCIL: implement min heap functions for graph search priority queue.
//   These functions work use the 'priority' field for elements in graph.


function insert(heap, elem) {
    var found = false; 

    // bubbles down
    for (var i = 0; i < heap.length; i++) { 
        if (heap[i].priority > elem.priority) { 
            // enqueue once location is found
            heap.splice(i, 0, elem); 
            found = true; 
            break; 
        } 
    } 

    //otherwise, highest prio - push it at the end 
    if (!found) { 
        heap.push(elem); 
    } 
}
function pop(heap) {
    //shift removes first element
    if (heap.length != 0) {
        return heap.shift();
    }
}