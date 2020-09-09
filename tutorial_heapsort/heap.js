/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Heap Sort Stencil | JavaScript support functions

    Quick JavaScript Code-by-Example Tutorial 
     
    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License 

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/


// create empty object 
minheaper = {}; 

// define insert function for min binary heap
function minheap_insert(heap, new_element) {
  heap.push(new_element)

  let i = heap.length - 1 
  while (i > 0) {
    var parent = Math.floor((i + 1) / 2) - 1;
      
    // swap if parent is greater than child
    if (heap[parent] > heap[i]) {
      var tmp = heap[parent];
      heap[parent] = heap[i];
      heap[i] = tmp;
    }
      
    i = parent;
  }
}

// assign insert function within minheaper object
minheaper.insert = minheap_insert;
/* Note: because the minheap_insert function is an object, we can assign 
      a reference to the function within the minheap object, which can be called
      as minheap.insert
*/

// define extract function for min binary heap
function minheap_extract(heap) {
  var min = heap[0], i = 0;
  //last element at top of head, then bubble down
  heap[0] = heap.pop();

  while (true) {
    var rightChild = (i + 1) * 2;
    var leftChild = rightChild - 1;
    var minChild;
    var toSwap = null;
    
    // get smallest child (right may not exist), and compare with parent
    if (!heap[rightChild]) {
      minChild = leftChild;
    } else {
      minChild = (heap[leftChild] < heap[rightChild]) ? leftChild : rightChild;
    }
    if (heap[i] > heap[minChild]) {
      toSwap = minChild;
    }
    
    // everything is in right place, so we done
    if (toSwap == null) {
      break;
    }
    
    var temp = heap[toSwap];
    heap[toSwap] = heap[i];
    heap[i] = temp;
    
    i = toSwap;
  }

  return min;
}
// assign extract function within minheaper object
minheaper.extract = minheap_extract;
    // STENCIL: ensure extract method is within minheaper object






