#ifndef IAR_ASTAR_PLANNER__NAVFN_HPP_
#define IAR_ASTAR_PLANNER__NAVFN_HPP_


#include <string.h>


namespace iar_astar_planner
{

    // cost defs
    #define COST_UNKNOWN_ROS 255  // 255 is unknown cost
    #define COST_OBS 254  // 254 for forbidden regions
    #define COST_OBS_ROS 253  // ROS values of 253 are obstacles

    // navfn cost values are set to
    // COST_NEUTRAL + COST_FACTOR * costmap_cost_value.
    // Incoming costmap cost values are in the range 0 to 252.
    // With COST_NEUTRAL of 50, the COST_FACTOR needs to be about 0.8 to
    // ensure the input values are spread evenly over the output range, 50
    // to 253.  If COST_FACTOR is higher, cost values will have a plateau
    // around obstacles and the planner will then treat (for example) the
    // whole width of a narrow hallway as equally undesirable and thus
    // will not plan paths down the center.

    #define COST_NEUTRAL 50  // Set this to "open space" value
    #define COST_FACTOR 0.8  // Used for translating costs in NavFn::setCostmap()

    #ifndef COSTTYPE
    #define COSTTYPE unsigned char  // Whatever is used...
    #endif


    #define POT_HIGHEST 1.0e10 //unassigned cell potential
    #define BUFFERSIZE 10000


    class NavFn
    {
    public:
        /**
         * @brief Constructs the planner
         * @param nx The x size of the map
         * @param ny The y size of the map
        */
        NavFn(int nx, int ny);
        ~NavFn();

        void setStart(int * start);
        void setGoal(int * goal);

    // protected:
        int nx_, ny_, ns_; /* size of grid, in pixels*/

        /** cell arrays*/
        COSTTYPE * costarr_; /**< cost array in 2D configuration space */
        float * potarr_; /**< potential array, navigation function potential */
        bool * pending_; /**< pending cells during propagation */

        float * pathx_, * pathy_;  /**< path points, as subpixel cell coordinates */
        int npath_;  /**< number of path points */
        int npathbuf_;  /**< size of pathx, pathy buffers */

        /** block update priority*/
        int * curPotentArr_;
        int n_curPotentArr_;
        int * nextPotentArr_;
        int n_nextPotentArr_;
        int * overflowPotentArr_;
        int n_overflowPotentArr_;
        float potentThreshInc_;
        float potentThresh_;

        int nobs_; /**< number of obstacle cells */
        void setNavArr(int nx, int ny);

        int goal_[2];
        int start_[2];

        /** 
         * @brief Set up the cost array for the planner from ROS
         * @param cmap The costmap
         * @param allow_unknown Whether or not the planner should be allowed to plan through unknown space
        */
        void setCostmap(const COSTTYPE * cmap, bool allow_unknown = true);
        bool setNavFn();
        #define push_cur(n) {if (n>=0 && n<ns_ && !pending_[n] &&\
                costarr_[n]<COST_OBS && n_curPotentArr_ < BUFFERSIZE)\
                {curPotentArr_[n_curPotentArr_++] = n;\
                pending_[n] = true;}}

        #define push_next(n) {if (n>=0 && n<ns_ && !pending_[n] &&\
                costarr_[n]<COST_OBS && n_nextPotentArr_ < BUFFERSIZE)\
                {nextPotentArr_[n_nextPotentArr_++] = n;\
                pending_[n] = true;}}

        #define push_over(n) {if (n>=0 && n<ns_ && !pending_[n] &&\
                costarr_[n]<COST_OBS && n_overflowPotentArr_ < BUFFERSIZE)\
                {overflowPotentArr_[n_overflowPotentArr_++] = n;\
                pending_[n] = true;}}

        bool propAstar(int cycles);
        void updateCell(int n);
        int calcPath(int n);

    };
}

#endif  