#ifndef IAR_AMCL__LIKELIHOOD_FIELD_MODEL_NODE_HPP_
#define IAR_AMCL__LIKELIHOOD_FIELD_MODEL_NODE_HPP_

#include "nav2_amcl/sensors/laser/laser.hpp"
namespace iar_amcl{
    class LikelihoodFieldModel : public nav2_amcl::Laser{
        public:
        LikelihoodFieldModel(double z_hit, double z_max, double z_rand, double sigma_hit, 
            double max_occ_dist, size_t max_beams, map_t * map);
        
        bool sensorUpdate(pf_t * pf, nav2_amcl::LaserData * data);


        private:
        static double sensorFunction(nav2_amcl::LaserData * data, pf_sample_set_t * set);
        double z_max_;

    };
}

#endif 