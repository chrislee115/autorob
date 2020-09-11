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
function updateRoundFactor() {
    tmp = eps;
    while (tmp < 0) {
        tmp = tmp * 10;
        ++roundFactor;
    }
}
var roundFactor = 0;
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
                x:xpos,y:ypos, // mapping to map coordinates
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
                insert(visit_queue, G[iind][jind]);
                G[iind][jind].distance = 0;
                G[iind][jind].visited = true;
                G[iind][jind].queued = true;
                G[iind][jind].priority = getDistToGoal(G[iind][jind]);
            }

        }
    }
}
function getDistToPoint(elem1, elem2) {
    return Math.pow(elem1.x - elem2.x, 2) + 
                            Math.pow(elem1.y - elem2.y, 2);
}
function getDistToGoal(elem) {
    return Math.pow(elem.x - q_goal[0], 2) + 
                            Math.pow(elem.y - q_goal[1], 2);
}
function addNeighbors(curr, neighbors) {
    for (var i = 0; i < 3; ++i) {
        for (var j = 0; j < 3; ++j) {
            if (i == 1 && j == 1) continue;
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
function iterateGraphSearch() {
    //probs not necessary
    if (search_visited == 0) { search_visited = 1; }
    if (visit_queue.length == 0) {
        return "failed";
    }
    if (getDistToGoal(visit_queue[0]) >= (eps / 10.0)) {
        var curr = pop(visit_queue);
        draw_2D_configuration([curr.x, curr.y], "visited");
        curr.visited = true;
        search_visited = search_visited + 1;
        neighbors = [];
        //collision detection in addneighbors
        addNeighbors(curr, neighbors);
        for (var i = 0; i < neighbors.length; ++i) {
            //this is g
            var changed = false;
            var tempDist = curr.distance + getDistToPoint(curr, neighbors[i]);
            if (neighbors[i].distance > tempDist) {
                changed = true;
                neighbors[i].distance = tempDist;
                neighbors[i].parent = curr;
                // g + h ( we dont store h )
                neighbors[i].priority = tempDist + getDistToGoal(neighbors[i]);
            }
            if (!neighbors[i].queued) {
                insert(visit_queue, neighbors[i]);
                if (changed) {
                    //this is very inefficient
                }
                neighbors[i].queued = true;
            }
            draw_2D_configuration([neighbors[i].x, neighbors[i].y], "queued");
        }
        fixInvariant(visit_queue);
        return "iterating";
    } else {
        drawHighlightedPathGraph(visit_queue[0]);
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
function insert(heap, elem) {
    // creating object from queue element 
    var contain = false; 

    // iterating through the entire 
    // item array to add element at the 
    // correct location of the Queue 
    for (var i = 0; i < heap.length; i++) { 
        if (heap[i].priority > elem.priority) { 
            // Once the correct location is found it is 
            // enqueued 
            heap.splice(i, 0, elem); 
            contain = true; 
            break; 
        } 
    } 

    // if the element have the highest priority 
    // it is added at the end of the queue 
    if (!contain) { 
        heap.push(elem); 
    } 
}
function pop(heap) {
    if (heap.length != 0) {
        return heap.shift();
    }
}
function fixInvariant(heap) {
    var tmp = []
    while (heap.length != 0) {
        tmp.push(pop(heap));
    }
    for (var i = 0; i < tmp.length; ++i) {
        insert(heap, tmp[i]);
    }
}
// class PrioQ {
//     constructor() {
//         this.items = [];
//     }
//     insert(elem) {
//         // creating object from queue element 
//         var contain = false; 
    
//         // iterating through the entire 
//         // item array to add element at the 
//         // correct location of the Queue 
//         for (var i = 0; i < this.items.length; i++) { 
//             if (this.items[i].priority > elem.priority) { 
//                 // Once the correct location is found it is 
//                 // enqueued 
//                 this.items.splice(i, 0, elem); 
//                 contain = true; 
//                 break; 
//             } 
//         } 
    
//         // if the element have the highest priority 
//         // it is added at the end of the queue 
//         if (!contain) { 
//             this.items.push(elem); 
//         } 
//     }
//     length() {
//         return this.items.length;
//     }
//     pop() {
//         if (!this.empty()) {
//             return this.items.shift();
//         }
//     }
//     front() {
//         if (!this.empty()) {
//             return this.items[0];
//         }
//     }
//     back() {
//         if (!this.empty()) {
//             return this.items[this.items.length - 1];
//         }
//     }
//     empty() {
//         return this.items.length == 0;
//     }
//     fixInvariant() {
//         var tmp = []
//         while (!this.empty()) {
//             tmp.push(this.pop());
//         }
//         for (var i = 0; i < tmp.length; ++i) {
//             this.insert(tmp[i]);
//         }
//     }
// }

    // STENCIL: implement min heap functions for graph search priority queue.
    //   These functions work use the 'priority' field for elements in graph.

