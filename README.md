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