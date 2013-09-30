#ifndef _KEYFRAMED_H_
#define _KEYFRAMED_H_

#include "node.h"

///@file igl/keyframed.h Keyframed Values. @ingroup igl
///@defgroup keyframed Keyframed Values
///@ingroup igl
///@{

/// Keyframed Value (interpolated like a bezier)
struct KeyframedValue : Node {
    const float         _epsilon = 0.00000001f; ///< epsilon
    
    vector<float>       times; ///< keyframe times
    vector<vec3f>       values; ///< keyframe values
    int                 degree = 1; ///< bezier interpolation degrees
    
    int segments() const { return values.size() / (degree+1); }
};

/// keyfamed animation interval
inline range1f keyframed_interval(KeyframedValue* keyframed) {
    return range1f( keyframed->times.front(), keyframed->times.back() );
}

/// Evaluates a keyframed spline
/// @param keyframed The KeyframedValue to evaluate
/// @param time The time at which to evaluate the keyframed spline
/// @return value The evaluated point
inline vec3f keyframed_value(KeyframedValue* keyframed, float time) {
    time = clamp(time,keyframed_interval(keyframed).min,keyframed_interval(keyframed).max-keyframed->_epsilon);

    int k = 0;

    // Perform sanity check
    if(time < keyframed->times[0]) {
        printf("keyframed_value: time error\n");
        exit(-1);
    }

    // Calculate the correct segment, k, into which time falls
    for(int j = 1; j < keyframed->times.size(); j++) {
        if(time >= keyframed->times[j - 1] && time < keyframed->times[j]) {
            k = j - 1;
            break;
        }
    }

    auto value = zero3f;

    // Calculate u
    auto u = (time - keyframed->times[k]) / (keyframed->times[k+1] - keyframed->times[k]);

    // Evaluate the spline segment
    for(int i = 0; i <= keyframed->degree; i++) {
        auto b = bernstein(u, i, keyframed->degree);
        auto p = keyframed->values[k * (keyframed->degree + 1) + i];
        value += b * p;
    }

    return value;
}

///@}

#endif
