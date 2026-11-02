#ifndef NAVIGATION_LIMITS_H
#define NAVIGATION_LIMITS_H
// Final arrival is encoder based; obstacle pass-through distances are millimetres.
namespace NavigationLimits {
constexpr int ArrivalToleranceTicks=5;
constexpr double TrackingReserveMm=20.0;
constexpr double PassThroughMarginMm=60.0;
}
#endif
