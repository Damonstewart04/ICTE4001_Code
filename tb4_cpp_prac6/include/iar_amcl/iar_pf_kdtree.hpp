#ifndef IAR_AMCL__PF__PF_KDTREE_HPP_
#define IAR_AMCL__PF__PF_KDTREE_HPP_

#include "nav2_amcl/pf/pf_vector.hpp"
#include "nav2_amcl/pf/pf_kdtree.hpp"

namespace iar_amcl
{
    // Destroy a tree
    void pf_kdtree_free(pf_kdtree_t * self);

    // Clear all entries from the tree
    void pf_kdtree_clear(pf_kdtree_t * self);

    // Insert a pose into the tree
    void pf_kdtree_insert(pf_kdtree_t * self, pf_vector_t pose, double value);

    // Cluster the leaves in the tree
    void pf_kdtree_cluster(pf_kdtree_t * self);

    // Determine the probability estimate for the given pose
    // extern double pf_kdtree_get_prob(pf_kdtree_t * self, pf_vector_t pose);

    // Determine the cluster label for the given pose
    int pf_kdtree_get_cluster(pf_kdtree_t * self, pf_vector_t pose);

}
#endif  // IAR_AMCL__PF__PF_KDTREE_HPP_
