#include "imprimis_navigation/speed_critic.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"


namespace mppi::critics
{

void SpeedCritic::initialize()
{
    auto getParam = parameters_handler_->getParamGetter(name_);

    getParam(cost_weight_, "cost_weight",1.0f);
    getParam(vx_max_, "vx_max",2.0f);
    getParam(threshold_to_consider_, "threshold_to_consider", 0.5f);
    getParam(obstacle_cost_threshold_, "obstacle_cost_threshold", 50.0f);
    getParam(near_obstacle_cost_, "near_obstacle_cost", 200.0f);



    RCLCPP_INFO(logger_, "SpeedCritic initialized with weight: %.2f", cost_weight_);

}

void SpeedCritic::score(CriticData & data){

    if (utils::withinPositionGoalTolerance(data.goal_checker, data.state.pose.pose, data.goal)){
    return;
}



    

    float effective_weight = cost_weight_ * data.model_dt;

    size_t num_trajectories = data.state.vx.shape(0);
    size_t num_timesteps = data.state.vx.shape(1);

    for (size_t i = 0; i < num_trajectories; i++){
        float mean_vx = 0.0f;
        for(size_t t=0; t < num_timesteps; t++){
            mean_vx += data.state.vx(i, t);
        }

        mean_vx /= static_cast<float>(num_timesteps);

        float penalty = vx_max_ - mean_vx;
        if (penalty < 0.0f) penalty = 0.0f;


        float max_cost  = 0.0f;
        for(size_t t=0; t < num_timesteps; t++){
            float x = data.trajectories.x(i, t);
            float y = data.trajectories.y(i, t);
            unsigned int mx, my;
            if (!costmap_ros_->getCostmap()->worldToMap(x, y, mx, my)) {
                continue;
            }
            float cost = static_cast<float>(costmap_ros_->getCostmap()->getCost(mx, my));
            if (cost > max_cost) max_cost = cost;
        
        
            if(cost > max_cost) max_cost = cost;
        }

        float scale = 1.0f;
        if (max_cost >= near_obstacle_cost_){
            scale = 0.0f;
        } 
        else if(max_cost > obstacle_cost_threshold_){
            scale = 1.0f - (max_cost - obstacle_cost_threshold_) /
            (near_obstacle_cost_ - obstacle_cost_threshold_);
        }


        data.costs(i) += effective_weight * scale * penalty;
        }
    }
}


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mppi::critics::SpeedCritic, mppi::critics::CriticFunction)