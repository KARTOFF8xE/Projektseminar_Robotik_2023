#include "decider.hpp"

#include <iostream>

decider::limit decider::get_limits(decider::limit limit_lf, std::vector<decider::limit> limits_hf) {
    const double time_thr = 1.0; // just don't change
    decider::limit limit;

    /**
     * Removes all Limits from the Buffer that lay too far in the past
    */
    int i = 0;
    while ((!limits_hf.empty()) && limits_hf.size() > i && (limit_lf.timestamp.seconds() - limits_hf[i].timestamp.seconds() > time_thr)) {
        std::cout << "[debug] on i=" << i << " got diff: " << limit_lf.timestamp.seconds() - limits_hf[i].timestamp.seconds() << std::endl;
        i++;
    }
    limits_hf.erase(limits_hf.begin(), limits_hf.begin() + i);
    
    /**
     * If there is no Value inside the Buffer:                                                      Nothing Happens
     * If there is one Value inside the Buffer & it is not too far in the Future:                   We collect it
     * If there are several Values inside The Buffer & the first one is not too far in the Future:  We collect the one that has a lower TimeDifference than the following Value
     * 
     * "Invalid" Values will always be removed, invalid Values are checked, but not selected Values
    */
    if ((!limits_hf.empty()) && (limit_lf.timestamp.seconds() - limits_hf[0].timestamp.seconds() > -time_thr)) {
        if (limits_hf.size() == 1) {
            limit = limits_hf[0];
            limits_hf.erase(limits_hf.begin());
        }
        while ((limit_lf.timestamp.seconds() - limits_hf[0].timestamp.seconds() > -time_thr) && limits_hf.size() >= 2) {
            if (std::abs(limit_lf.timestamp.seconds() - limits_hf[0].timestamp.seconds()) > std::abs(limit_lf.timestamp.seconds() - limits_hf[1].timestamp.seconds())) {
                limits_hf.erase(limits_hf.begin());
                continue;
            } else {
                limit = limits_hf[0];
                limits_hf.erase(limits_hf.begin());
                break;
            }
        }
    }

    /**
     * If we only have a valid Limit from the Buffer:                                   take it
     * If we have collected a Limit from the Buffer and an existing one for the Check:  combine them
     * If we have no Limit from the Buffer, but one for the Check:                      take the one of the Check
     * If we have no Limits:                                                            Impossible to find a Limit this Time
    */
    if (!(limit_lf.limit > 0)) {
        return limit;
    }
    limit = (limit.limit > 0) ? decider::limit{(limit_lf.limit * limit.limit) / 2, limit_lf.timestamp} : limit_lf;

    return limit;
}