#include <doctest/doctest.h>
#include "fusion_orientation_estimator.h"

namespace {
double angle(const Eigen::Quaternionf &a, const Eigen::Quaternionf &b) {
    return a.normalized().angularDistance(b.normalized());
}
}

TEST_CASE("Fusion adapter reset matches Fusion with the public gain settings throughout startup") {
    for (auto convention : {AxesConvention::Nwu, AxesConvention::Enu, AxesConvention::Ned}) {
        FusionOrientationEstimator filter;
        filter.reset(convention,.05f);
        FusionAhrs reference{};
        FusionAhrsInitialise(&reference);
        FusionAhrsSettings settings{convention==AxesConvention::Nwu ? FusionConventionNwu :
            convention==AxesConvention::Enu ? FusionConventionEnu : FusionConventionNed,
            .05f,90.f,90.f,0};
        FusionAhrsSetSettings(&reference,&settings);
        for (int i=0;i<2200;++i) {
            const Eigen::Vector3f acc(.2f,0,1);
            filter.update({0,0,2},acc,.002f);
            FusionVector gyro{0,0,2}, acceleration{.2f,0,1}, zero{0,0,0};
            FusionAhrsUpdate(&reference,gyro,acceleration,zero,.002f);
            const auto q=FusionAhrsGetQuaternion(&reference);
            const auto actual=filter.orientation();
            CHECK(actual.w()==q.element.w); CHECK(actual.x()==q.element.x);
            CHECK(actual.y()==q.element.y); CHECK(actual.z()==q.element.z);
        }
        filter.reset(convention,.05f);
        CHECK(angle(filter.orientation(),Eigen::Quaternionf::Identity())<1e-6);
    }
}
