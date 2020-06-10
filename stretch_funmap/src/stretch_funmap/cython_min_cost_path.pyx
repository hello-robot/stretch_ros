import numpy as np
import heapq as he

cimport numpy as np
from cpython cimport bool
import cython

# manual compilation
# cython ./cython_min_cost_path.pyx 
# gcc -shared -pthread -fPIC -fwrapv -O2 -Wall -fno-strict-aliasing -I/usr/include/python2.7 -o cython_min_cost_path.so cython_min_cost_path.c

# other compiler commands in attempt to improve efficiency
# gcc -shared -pthread -fPIC -fwrapv -ffast-math -O3 -Wall -fno-strict-aliasing -I/usr/include/python2.7 -o cython_min_cost_path.so cython_min_cost_path.c

DTYPE0 = np.float32
ctypedef np.float32_t DTYPE0_t

DTYPE1 = np.uint8
ctypedef np.uint8_t DTYPE1_t

DTYPE2 = np.int8
ctypedef np.int8_t DTYPE2_t

DTYPE3 = np.int64
ctypedef np.int64_t DTYPE3_t

DTYPE4 = np.uint16
ctypedef np.uint16_t DTYPE4_t

@cython.boundscheck(False) # turn off bounds-checking for entire function
@cython.wraparound(False)  # turn off negative index wrapping for entire function

def cython_costs():
    cdef float min_cost = 1.0
    cdef float max_cost = 100.0
    cdef float none_shall_pass = -1.0
    return min_cost, max_cost, none_shall_pass


def cython_cost_map(np.ndarray[DTYPE0_t, ndim=2] distance_image):
    # Each pixel coordinate, [xi, yi], in a path has an associated
    # cost, c[xi,yi]. The cost of a path is the sum of these
    # costs, Sum(cost(path[i])).
    #
    # The cost, c, for each pixel coordinate, [xi, yi], is a
    # function of the distance value, d, for the pixel coordinate
    # in the provided distance map, d = distance_image[xi,yi].
    #
    # For a pixel coordinate, the associated cost, c(d) has the
    # following properties:
    #
    # min_cost = 1.0
    # max_cost = 100.0
    #
    # As d approaches infinity, the associated cost, c(d),
    # approaches min_cost.
    #
    # As d approaches 0, the associated cost, c(d), approaches
    # max_cost.
    #
    # When d <= 0.0, the pixel coordinate is not allowed to be
    # included in a valid path. This is comparable to c(d) =
    # infinity.
    #
    # For d > 0.0, the associated cost,
    # c(d) = min_cost + ((max_cost - min_cost)/(d + 1.0))
    #
    # This cost function prefers paths that are conservative,
    # keeping the robot far from obstacles and often moving along
    # the medial axes of shapes. This fits well with roadmaps that
    # can be created from the ridges of distance maps.

    cdef float min_cost
    cdef float max_cost
    cdef float none_shall_pass
    min_cost, max_cost, none_shall_pass = cython_costs()
    
    cdef float max_cost_minus_min_cost = max_cost - min_cost
    
    cdef int height = distance_image.shape[0]
    cdef int width = distance_image.shape[1]

    cdef float val
    cdef int x
    cdef int y
    
    # cost
    # as distance -> inf, cost -> min_cost
    # distance -> min_robot_distance -> cost -> typical_max_cost + min_cost
    # distance > min_robot_distance -> cost = none_shall_pass

    cdef float min_cost_in_image = max_cost
    cdef np.ndarray[DTYPE0_t, ndim=2] cost_image = np.empty((height, width), dtype=DTYPE0)
    for y in range(height):
        for x in range(width):
            val = distance_image[y,x]
            if val <= 0.0:
                val = none_shall_pass
            else: 
                val = min_cost + (max_cost_minus_min_cost/(val + 1.0))
                if val < min_cost_in_image:
                    min_cost_in_image = val
            cost_image[y,x] = val
            
            
    # create impassable border at the edges of the image to keep the
    # search in bounds
    cost_image[:,0] = none_shall_pass
    cost_image[:, width-1] = none_shall_pass
    cost_image[0,:] = none_shall_pass
    cost_image[height-1,:] = none_shall_pass
    return cost_image

def cython_4_way_connectivity():
    #   0 
    # 1   2
    #   3

    cdef np.ndarray[DTYPE1_t, ndim=1] opposite_direction = np.zeros((4,), dtype=DTYPE1)
    opposite_direction[0] = 3
    opposite_direction[1] = 2
    opposite_direction[2] = 1
    opposite_direction[3] = 0
    
    cdef np.ndarray[DTYPE2_t, ndim=2] direction_to_xy = np.zeros((4, 2), dtype=DTYPE2)
    direction_to_xy[0] = [0, -1]
    direction_to_xy[1] = [-1, 0]
    direction_to_xy[2] = [1, 0]
    direction_to_xy[3] = [0, 1]

    # 4 cardinal directions (up, left, right, down)
    cdef np.ndarray[DTYPE2_t, ndim=1] directions = np.array([0, 1, 2, 3], dtype=DTYPE2)

    cdef int no_direction = 255
    cdef int start_marker = 250

    return directions, no_direction, start_marker, direction_to_xy, opposite_direction


def cython_8_way_connectivity():
    # 0 1 2
    # 3   4
    # 5 6 7

    cdef np.ndarray[DTYPE1_t, ndim=1] opposite_direction = np.zeros((10,), dtype=DTYPE1)
    opposite_direction[0] = 7
    opposite_direction[1] = 6
    opposite_direction[2] = 5
    opposite_direction[3] = 4
    opposite_direction[4] = 3
    opposite_direction[5] = 2
    opposite_direction[6] = 1
    opposite_direction[7] = 0

    cdef np.ndarray[DTYPE2_t, ndim=2] direction_to_xy = np.zeros((10, 2), dtype=DTYPE2)
    direction_to_xy[0] = [-1, -1]
    direction_to_xy[1] = [ 0, -1]
    direction_to_xy[2] = [ 1, -1]
    direction_to_xy[3] = [-1,  0]
    direction_to_xy[4] = [ 1,  0]
    direction_to_xy[5] = [-1,  1]
    direction_to_xy[6] = [ 0,  1]
    direction_to_xy[7] = [ 1,  1]

    # 8 directions
    cdef np.ndarray[DTYPE2_t, ndim=1] directions = np.array([0, 1, 2, 3, 4, 5, 6, 7], dtype=DTYPE2)

    cdef int no_direction = 255
    cdef int start_marker = 250
    
    return directions, no_direction, start_marker, direction_to_xy, opposite_direction


def cython_direction_image_to_path(np.ndarray[DTYPE1_t, ndim=2] direction_image,
                                   np.ndarray[DTYPE3_t, ndim=1] end_xy):

    cdef np.ndarray[DTYPE1_t, ndim=1] opposite_direction
    cdef np.ndarray[DTYPE2_t, ndim=2] direction_to_xy
    cdef np.ndarray[DTYPE2_t, ndim=1] directions
    cdef int no_direction
    cdef int start_marker
    
    directions, no_direction, start_marker, direction_to_xy, opposite_direction = cython_4_way_connectivity()

    cdef bool at_end = False
    cdef float path_cost = -1.0
    cdef int end_x = end_xy[0]
    cdef int end_y = end_xy[1]
    cdef int n
    cdef np.ndarray[DTYPE2_t, ndim=1] direction

    if direction_image[end_y, end_x] == no_direction:
        path = None
        return path
    
    # path found
    current_x, current_y = end_xy
    path = []
    while direction_image[current_y, current_x] != start_marker:
        path.append([current_x, current_y])
        n = direction_image[current_y, current_x]
        current_x = direction_to_xy[n, 0] + current_x
        current_y = direction_to_xy[n, 1] + current_y
    path.append([current_x, current_y])
    path.reverse()
        
    return path


def cython_min_cost_path(np.ndarray[DTYPE0_t, ndim=2] distance_image,
                         np.ndarray[DTYPE3_t, ndim=1] start_xy,
                         np.ndarray[DTYPE3_t, ndim=1] end_xy):
    # Searches for a minimum cost path using 4-way connectivity and no
    # heuristic. This tracks the ridges in distance maps well. It does
    # not perform well if the distance map is clipped to have a flat
    # section, indicating a region where straight line motions would
    # be optimal due to path length. In these situations, the paths
    # typically consist of pure vertical and horizontal segments
    # rather than straight lines between relevant points. Tests with
    # 8-way connectivity and heuristics improved flat region
    # performance, but degraded ridge tracking and still produced flat
    # region paths consisting of sections the same motion repeated in
    # a manner that did not approximate straight line very well.

    # The returned path consists of a list of pixel coordinates in
    # the distance_map, path = [[x0,y0], [x1,y1], ...], that define a
    # minimum cost path from pixel coordinate start_xy to pixel
    # coordinate end_xy. 

    cost_image = cython_cost_map(distance_image)
    
    cdef float min_cost
    cdef float max_cost
    cdef float none_shall_pass
    min_cost, max_cost, none_shall_pass = cython_costs()

    cdef np.ndarray[DTYPE1_t, ndim=1] opposite_direction
    cdef np.ndarray[DTYPE2_t, ndim=2] direction_to_xy
    cdef np.ndarray[DTYPE2_t, ndim=1] directions
    cdef int no_direction
    cdef int start_marker
    
    directions, no_direction, start_marker, direction_to_xy, opposite_direction = cython_4_way_connectivity()

    cdef np.ndarray[DTYPE1_t, ndim=2] direction_image = np.full_like(distance_image, no_direction, dtype=DTYPE1)
    
    h = []
    cdef bool at_end = False
    cdef int current_x = start_xy[0]
    cdef int current_y = start_xy[1]
    cdef int end_x = end_xy[0]
    cdef int end_y = end_xy[1]
    cdef int came_from_here = 0
    cdef float total_cost = 0.0
    cdef float new_cost
    cdef float new_total
    cdef int backwards_direction
    cdef float path_cost = -1.0
    cdef int k
    
    he.heappush(h, (total_cost, current_x, current_y, came_from_here))

    while h:
        total_cost, current_x, current_y, came_from_here = he.heappop(h)
        for k in directions:
            if k != came_from_here:
                x = direction_to_xy[k,0] + current_x
                y = direction_to_xy[k,1] + current_y
                new_cost = cost_image[y,x]
                if new_cost > none_shall_pass:
                    new_total = total_cost + new_cost
                    cost_image[y,x] = none_shall_pass
                    backwards_direction = opposite_direction[k]
                    direction_image[y,x] = backwards_direction
                    if (current_x == end_x) and (current_y == end_y):
                        path_cost = new_total
                        at_end = True
                        break
                    he.heappush(h, (new_total, x, y, backwards_direction))

    cdef int start_x = start_xy[0]
    cdef int start_y = start_xy[1]
    direction_image[start_y, start_x] = start_marker

    if not at_end:
        # no path found
        path = None
        return path
    
    path = cython_direction_image_to_path(direction_image, end_xy)
    return path
    

def cython_all_paths(np.ndarray[DTYPE0_t, ndim=2] distance_image,
                     np.ndarray[DTYPE3_t, ndim=1] start_xy):
    # Produces representations that allow for the efficient
    # construction of minimum paths to any location from start_xy.
    # The minimum paths tend to track the ridges in the provided
    # distance map.

    # This does not perform well if the distance map is clipped to
    # have a flat section, indicating a region where straight line
    # motions would be optimal due to path length. In these
    # situations, the paths typically consist of pure vertical and
    # horizontal segments rather than straight lines between relevant
    # points. Tests with 8-way connectivity and heuristics improved
    # flat region performance, but degraded ridge tracking and still
    # produced flat region paths consisting of sections the same
    # motion repeated in a manner that did not approximate straight
    # line very well.
    #
    # Each pixel coordinate, [xi, yi], in a path has an associated
    # cost, c[xi,yi]. The cost of a path is the sum of these
    # costs, Sum(cost(path[i])).
    #
    
    cost_image = cython_cost_map(distance_image)
    
    cdef float min_cost
    cdef float max_cost
    cdef float none_shall_pass
    min_cost, max_cost, none_shall_pass = cython_costs()

    cdef np.ndarray[DTYPE1_t, ndim=1] opposite_direction
    cdef np.ndarray[DTYPE2_t, ndim=2] direction_to_xy
    cdef np.ndarray[DTYPE2_t, ndim=1] directions
    cdef int no_direction
    cdef int start_marker
    directions, no_direction, start_marker, direction_to_xy, opposite_direction = cython_4_way_connectivity()
    
    cdef np.ndarray[DTYPE1_t, ndim=2] direction_image = np.full_like(distance_image, no_direction, dtype=DTYPE1)
    cdef np.ndarray[DTYPE4_t, ndim=2] path_length_image = np.zeros_like(distance_image, dtype=DTYPE4)
    
    h = []
    cdef int current_x = start_xy[0]
    cdef int current_y = start_xy[1]
    cdef int came_from_here = 0
    cdef float total_cost = 0.0
    cdef float new_cost
    cdef float new_total
    cdef int backwards_direction
    cdef float path_cost = -1.0
    cdef int k
    cdef int path_length = 0
    cdef int new_path_length
    
    he.heappush(h, (total_cost, path_length, current_x, current_y, came_from_here))

    while h:
        total_cost, path_length, current_x, current_y, came_from_here = he.heappop(h)
        for k in directions:
            if k != came_from_here:
                x = direction_to_xy[k,0] + current_x
                y = direction_to_xy[k,1] + current_y
                new_cost = cost_image[y,x]
                if new_cost > none_shall_pass:
                    new_total = total_cost + new_cost
                    cost_image[y,x] = none_shall_pass
                    backwards_direction = opposite_direction[k]
                    direction_image[y,x] = backwards_direction
                    new_path_length = path_length + 1
                    path_length_image[y,x] = new_path_length
                    he.heappush(h, (new_total, new_path_length, x, y, backwards_direction))

    cdef int start_x = start_xy[0]
    cdef int start_y = start_xy[1]
    direction_image[start_y, start_x] = start_marker

    return path_length_image, direction_image
