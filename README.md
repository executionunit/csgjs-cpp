# csgjs-cpp

Addition work to the dabroz port of the initial Javascript code here: https://github.com/evanw/csg.js/.

## Changes

List of changes from the initial port here: https://github.com/dabroz/csgjs-cpp

* turned in to header only
* remove inline static from functions, now just inline
* add negate operator for csgjs_vector
* `csgjs_EPSILON` set to 0.0001 (from 0.00001) as we're using floats.
* port `csgsmodel_cube`, `csgmodel_sphere` and `csgsmodel_cyliner` from the JS library
* bring in the gourd model for comparisons tests with JS version.
* Add color in to the csgjs_vertex structure and interpolation. Better exports.
* Can define CSGJSCPP_REAL to double to force doubles for everything, defaults to float.

base point

    gourd union cyl 289372ms
    gourd intersect cyl 143341ms
    gourd subtract cyl 242513ms
    cyl subtract gourd 142125ms

types.reserve()

    gourd union cyl 226712ms
    gourd intersect cyl 109099ms
    gourd subtract cyl 185550ms
    cyl subtract gourd 118081ms

replace std::list with std::deque

    gourd union cyl 218010ms
    gourd intersect cyl 104212ms
    gourd subtract cyl 192249ms
    cyl subtract gourd 112567ms

remove vector copy from many looping ops.

    gourd union cyl 192250ms
    gourd intersect cyl 91525ms
    gourd subtract cyl 159293ms
    cyl subtract gourd 99925ms

in split polygon: do not cache the polygon type as creating the memory to cache
it takes time and often the cache idn't needed. Just repeat the maths.

    gourd union cyl 178445ms
    gourd intersect cyl 80472ms
    gourd subtract cyl 138508ms
    cyl subtract gourd 90476ms

last cleanup

    gourd union cyl 163780ms
    gourd intersect cyl 84397ms
    gourd subtract cyl 139961ms
    cyl subtract gourd 100031ms

add color in to csgjs_vertex. interestingly expensive.

    gourd union cyl 200411ms
    gourd intersect cyl 107755ms
    gourd subtract cyl 168332ms
    cyl subtract gourd 102270ms

remove UV as not used.

    gourd union cyl 184774ms
    gourd intersect cyl 105220ms
    gourd subtract cyl 167227ms
    cyl subtract gourd 102725ms