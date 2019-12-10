/* ***********************************
 * Author: Andrin Jenal
 * Supervisor: Marcel Lancelle
 * Department: ETH Zürich
 * Copyright: 2013 ETH Zürich
 * File: FeatureTrackingParams.hpp
 * **********************************/

#ifndef VIDEOSTAB_FEATURETRACKING_PARAMS_HPP
#define VIDEOSTAB_FEATURETRACKING_PARAMS_HPP

struct FeatureTrackingParams {
    int maxNumFeat; // maximum number of features
    double qualLev; // quality Level
    double minDist; // min distance
    int blSize; // block size
    bool harrCor; // harris corner
};

#endif // VIDEOSTAB_FEATURETRACKING_HPP
