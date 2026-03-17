#ifndef IMPRIMIS_NAVIGATION__SPEED_CRITIC_HPP_
#define IMPRIMIS_NAVIGATION_SPEED_CRITIC_HPP_


#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/models/trajectories.hpp"
#include "nav2_mppi_controller/models/path.hpp"



namespace mppi::critics
{

    class SpeedCritic : public CriticFunction
    {

        public:
            void initialize() override;
            void score(CriticData & data) override;


        protected:
            float cost_weight_;
            float vx_max_;
            float threshold_to_consider_;
            float obstacle_cost_threshold_;
            float near_obstacle_cost_;





    };

}

#endif