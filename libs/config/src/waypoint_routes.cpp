#include "waypoint_routes.h"

COMMON::Waypoint testSquare[5] = {   ///example waypoint list for testing
        {0.0, 0.0, 0.0, 0.25},
        {0.0, 1.0, 0.0, 0.25}, 
        {1.0, 1.0, 0.0, 0.25},
        {1.0, 0.0, 0.0, 0.25},
        {0.0, 0.0, 0.0, 0.25}
};

COMMON::Waypoint lavaRoute[43] = {   //assumes starting with rear of bot level with end of course
        {0.000, 0.000,  0.000, 0.100},
        {0.000, 0.000,  0.000, 0.1},
        {0.000, 0.078,  0.000, 0.1},
        {0.000, 0.235,  0.000, 0.100},
        {0.000, 0.435,  0.000, 0.100},
        {0.000, 0.635,  0.000, 0.100},
        {0.000, 0.835,  0.000, 0.100},
        {0.000, 1.035,  0.000, 0.1000},
        {0.000, 1.235,  0.000, 0.100},
        {0.000, 1.435,  0.000, 0.1000},
        {0.000, 1.635,  -0.017, 0.100},
        {-0.003, 1.835,  -0.140, 0.1000},
        {-0.031, 2.033,  -0.332, 0.100},
        {-0.096, 2.223,  -0.489, 0.1000},
        {-0.190, 2.399,  -0.646, 0.100},
        {-0.311, 2.559,  -0.555, 0.1000},
        {-0.416, 2.729,  -0.396, 0.100},
        {-0.493, 2.913,  -0.237, 0.1000},
        {-0.540, 3.108,  -0.075, 0.100},
        {-0.555, 3.307,  0.000, 0.1000},
        {-0.555, 3.507,  0.000, 0.100},
        {-0.555, 3.707,  0.017, 0.1000},
        {-0.552, 3.907,  0.087, 0.100},
        {-0.534, 4.106,  0.192, 0.1000},
        {-0.496, 4.303,  0.419, 0.100},
        {-0.415, 4.485,  0.541, 0.1000},
        {-0.312, 4.657,  0.611, 0.100},
        {-0.197, 4.821,  0.524, 0.1000},
        {-0.097, 4.994,  0.314, 0.100},
        {-0.035, 5.184,  0.105, 0.1000},
        {-0.014, 5.383,  0.000, 0.100},
        {-0.014, 5.583,  0.000, 0.1000},
        {-0.014, 5.783,  0.000, 0.100},
        {-0.014, 5.983,  0.000, 0.1000},
        {-0.014, 6.183,  0.000, 0.100},
        {-0.014, 6.383,  0.000, 0.1000},
        {-0.014, 6.583,  0.000, 0.100},
        {-0.014, 6.783,  0.000, 0.1000},
        {-0.014, 6.983,  0.000, 0.100},
        {-0.014, 7.183,  0.000, 0.1},
        {-0.038, 7.4,  0.000, 0.0},
        {-0.038, 7.5,  0.000, 0.0},
        {-0.023, 7.6,  0.000, 0.0}
};

COMMON::Waypoint ecodisasterRoute[98] = {   //dumb field-ploughing
        {0.000, -0.750,  0.000, 0.05},
        {0.000, -0.550,  0.000, 0.05},
        {0.000, -0.350,  0.000, 0.05},
        {0.000, -0.150,  0.000, 0.05},
        {0.000, 0.050,  0.000, 0.05},
        {0.000, 0.250,  0.000, 0.05},
        {0.000, 0.450,  0.000, 0.05},
        {0.000, 0.550,  0.000, 0.05},
        {0.133, 0.727,  0.000, 0.05},
        {0.407, 0.748,  0.000, 0.05},
        {0.637, 0.643,  0.000, 0.05},
        {0.778, 0.435,  0.000, 0.05},
        {0.742, 0.163,  0.000, 0.05},
        {0.675, 0.050,  0.000, 0.05},
        {0.675, -0.150,  0.000, 0.05},
        {0.675, -0.350,  0.000, 0.05},
        {0.675, -0.550,  0.000, 0.05},
        {0.675, -0.650,  0.000, 0.05},
        {0.542, -0.827,  0.000, 0.05},
        {0.268, -0.848,  0.000, 0.05},
        {0.068, -0.750,  0.000, 0.05},
        {-0.187, -0.750,  0.000, 0.05},
        {-0.328, -0.535,  0.000, 0.05},
        {-0.292, -0.263,  0.000, 0.05},
        {-0.225, -0.150,  0.000, 0.05},
        {-0.225, 0.050,  0.000, 0.05},
        {-0.225, 0.250,  0.000, 0.05},
        {-0.225, 0.450,  0.000, 0.05},
        {-0.225, 0.550,  0.000, 0.05},
        {-0.092, 0.727,  0.000, 0.05},
        {0.182, 0.748,  0.000, 0.05},
        {0.412, 0.643,  0.000, 0.05},
        {0.553, 0.435,  0.000, 0.05},
        {0.517, 0.163,  0.000, 0.05},
        {0.450, 0.050,  0.000, 0.05},
        {0.450, -0.150,  0.000, 0.05},
        {0.450, -0.350,  0.000, 0.05},
        {0.450, -0.550,  0.000, 0.05},
        {0.450, -0.650,  0.000, 0.05},
        {0.317, -0.827,  0.000, 0.05},
        {0.043, -0.848,  0.000, 0.05},
        {-0.157, -0.750,  0.000, 0.05},
        {-0.412, -0.750,  0.000, 0.05},
        {-0.553, -0.535,  0.000, 0.05},
        {-0.517, -0.263,  0.000, 0.05},
        {-0.450, -0.150,  0.000, 0.05},
        {-0.450, 0.050,  0.000, 0.05},
        {-0.450, 0.250,  0.000, 0.05},
        {-0.450, 0.450,  0.000, 0.05},
        {-0.450, 0.550,  0.000, 0.05},
        {-0.317, 0.727,  0.000, 0.05},
        {-0.043, 0.748,  0.000, 0.05},
        {0.187, 0.643,  0.000, 0.05},
        {0.328, 0.435,  0.000, 0.05},
        {0.292, 0.163,  0.000, 0.05},
        {0.225, 0.050,  0.000, 0.05},
        {0.225, -0.150,  0.000, 0.05},
        {0.225, -0.350,  0.000, 0.05},
        {0.225, -0.550,  0.000, 0.05},
        {0.225, -0.650,  0.000, 0.05},
        {0.092, -0.827,  0.000, 0.05},
        {-0.182, -0.848,  0.000, 0.05},
        {-0.382, -0.750,  0.000, 0.05},
        {-0.637, -0.750,  0.000, 0.05},
        {-0.778, -0.535,  0.000, 0.05},
        {-0.742, -0.263,  0.000, 0.05},
        {-0.675, -0.150,  0.000, 0.05},
        {-0.675, 0.050,  0.000, 0.05},
        {-0.675, 0.250,  0.000, 0.05},
        {-0.675, 0.450,  0.000, 0.05},
        {-0.675, 0.650,  0.000, 0.05},
        {-0.450, 0.850,  0.000, 0.05},
        {-0.400, 0.950,  0.000, 0.00},
        {-0.400, 1.050,  0.000, 0.00}
};

COMMON::Waypoint escapeRouteRoute[36] = {   //average/centreline route
        {0.000, 0.000,  0.000, 0.05},
        {0.000, 0.010,  0.000, 0.05},
        {0.000, 0.098,  0.000, 0.05},
        {0.000, 0.265,  0.000, 0.05},
        {0.000, 0.465,  0.000, 0.05},
        {0.000, 0.665,  0.000, 0.05},
        {0.000, 0.865,  0.000, 0.05},
        {0.000, 1.065,  3.142, 0.05},
        {0.000, 1.036,  0.374, 0.05},
        {0.086, 1.256,  1.013, 0.05},
        {0.295, 1.386,  1.571, 0.05},
        {0.495, 1.386,  1.571, 0.05},
        {0.695, 1.386,  1.944, 0.05},
        {0.915, 1.300,  2.866, 0.05},
        {0.989, 1.036,  3.142, 0.05},
        {0.989, 0.836,  3.142, 0.05},
        {0.989, 0.636,  3.142, 0.05},
        {0.989, 0.589,  2.768, 0.05},
        {1.075, 0.369,  2.129, 0.05},
        {1.284, 0.239,  1.571, 0.05},
        {1.484, 0.239,  1.571, 0.05},
        {1.684, 0.239,  1.197, 0.05},
        {1.904, 0.325,  0.275, 0.05},
        {1.978, 0.589,  0.000, 0.05},
        {1.978, 0.789,  0.000, 0.05},
        {1.978, 0.989,  0.000, 0.05},
        {1.978, 1.036,  0.374, 0.05},
        {2.064, 1.256,  1.013, 0.05},
        {2.273, 1.386,  1.571, 0.05},
        {2.473, 1.386,  1.571, 0.05},
        {2.673, 1.386,  1.571, 0.05},
        {2.773, 1.386,  1.571, 0.05},
        {2.933, 1.386,  1.571, 0.05},
        {3.016, 1.386,  1.571, 0.05},
        {3.019, 1.386,  1.588, 0.0},
        {3.30, 1.386,  1.588, 0.0}
};

COMMON::Waypoint minesweeperRoute[24] = {   //wavy square route
        {-0.380, 0.000,  -0.261, 0.05},
        {-0.420, 0.150,  -0.405, 0.05},
        {-0.480, 0.290,  0.245, 0.05},
        {-0.440, 0.450,  1.360, 0.05},
        {-0.300, 0.480,  1.976, 0.05},
        {-0.160, 0.420,  1.816, 0.05},
        {0.000, 0.380,  1.326, 0.05},
        {0.160, 0.420,  1.166, 0.05},
        {0.300, 0.480,  1.782, 0.05},
        {0.440, 0.450,  2.897, 0.05},
        {0.480, 0.290,  -2.810, 0.05},
        {0.420, 0.150,  3.006, 0.05},
        {0.380, 0.000,  2.810, 0.05},
        {0.420, -0.150,  3.075, 0.05},
        {0.480, -0.290,  -2.897, 0.05},
        {0.440, -0.450,  -1.782, 0.05},
        {0.300, -0.480,  -1.166, 0.05},
        {0.160, -0.420,  -1.326, 0.05},
        {0.000, -0.380,  -1.816, 0.05},
        {-0.160, -0.420,  -1.976, 0.05},
        {-0.300, -0.480,  -1.360, 0.05},
        {-0.440, -0.450,  -0.245, 0.05},
        {-0.480, -0.290,  1.027, 0.05},
        {-0.440, -0.150,  1.242, 0.05},
};
