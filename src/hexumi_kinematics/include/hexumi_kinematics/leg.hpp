#pragma once
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>

namespace Leg {
    enum leg_IDs {
        L1 = 0,
        L2 = 1,
        L3 = 2,
        R1 = 3,
        R2 = 4,
        R3 = 5
    };

    class Leg {
    public:
        Leg(leg_IDs);

        static float coxa_length;
        static float femur_length;
        static float tibia_length;

        static float get_coxa();
        static float get_femur();
        static float get_tibia();

    private:
        leg_IDs leg_id;
        float origin_radius;
        float origin_angle;
    };
}