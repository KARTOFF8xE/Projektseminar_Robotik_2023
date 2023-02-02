#include "decider.hpp"

decider::limit decider::get_limits(decider::limit limit_1, std::vector<decider::limit> limits_2) {
    const int time_thr = 1; // just don't change
    decider::limit limit;

    /**
     * Removes all Limits from the Buffer that lay too far in the past
    */
    while ((!limits_2.empty()) && (limit_1.timestamp.seconds() - limits_2[0].timestamp.seconds() > time_thr)) {
        limits_2.erase(limits_2.begin());
    }
    
    /**
     * If there is no Value inside the Buffer:                                                      Nothing Happens
     * If there is one Value inside the Buffer & it is not too far in the Future:                   We collect it
     * If there are several Values inside The Buffer & the first one is not too far in the Future:  We collect the one that has a lower TimeDifference than the following Value
     * 
     * "Invalid" Values will always be removed, invalid Values are checked, but not selected Values
    */
    if ((!limits_2.empty()) && (limit_1.timestamp.seconds() - limits_2[0].timestamp.seconds() > -time_thr)) {
        if (limits_2.size() == 1) {
            limit = limits_2[0];
            limits_2.erase(limits_2.begin());
        }
        while ((limit_1.timestamp.seconds() - limits_2[0].timestamp.seconds() > -time_thr) && limits_2.size() >= 2) {
            if (std::abs(limit_1.timestamp.nanoseconds() - limits_2[0].timestamp.nanoseconds()) > std::abs(limit_1.timestamp.nanoseconds() - limits_2[1].timestamp.nanoseconds())) {
                limits_2.erase(limits_2.begin());
                continue;
            } else {
                limit = limits_2[0];
                limits_2.erase(limits_2.begin());
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
    if (!(limit_1.limit > 0)) {
        return limit;
    }
    limit = (limit.limit > 0) ? decider::limit{(limit_1.limit * limit.limit) / 2, limit_1.timestamp} : limit_1;

    return limit;
}