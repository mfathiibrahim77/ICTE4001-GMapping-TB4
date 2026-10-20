#include "iar_astar_planner/navfn.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <algorithm>

namespace iar_astar_planner
{
    NavFn::NavFn(int nx, int ny)
    {
          // create cell arrays
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[NavFn] Constructor");
        costarr_ = NULL;
        potarr_ = NULL;
        pending_ = NULL;
        setNavArr(nx, ny);

        curPotentArr_ = new int[BUFFERSIZE];
        nextPotentArr_ = new int[BUFFERSIZE];
        overflowPotentArr_ = new int[BUFFERSIZE];

        potentThreshInc_ = 2 * COST_NEUTRAL;


        npathbuf_ = npath_ = 0;
        pathx_ = pathy_ = NULL;
        
    }
    NavFn::~NavFn()
    {
        if (costarr_) {
            delete[] costarr_;
        }
        if (potarr_) {
            delete[] potarr_;
        }
        if (curPotentArr_){
            delete[] curPotentArr_;
        }
        if(nextPotentArr_){
            delete[] nextPotentArr_;
        }
        if(overflowPotentArr_){
            delete[] overflowPotentArr_;
        }
        if (pending_) {
            delete[] pending_;
        }

        if (pathx_) {
            delete[] pathx_;
        }
        if (pathy_) {
            delete[] pathy_;
        }
    }

    void NavFn::setNavArr(int nx, int ny)
    {
        nx_ = nx;
        ny_ = ny;
        ns_ = nx * ny;

        

        if (costarr_) {
            delete[] costarr_;
        }
        if (potarr_) {
            delete[] potarr_;
        }
        if (pending_) {
            delete[] pending_;
        }

        costarr_ = new COSTTYPE[ns_];  // cost array, 2d config space
        memset(costarr_, 0, ns_ * sizeof(COSTTYPE));
        potarr_ = new float[ns_];  // navigation potential array
        pending_ = new bool[ns_];
        memset(pending_, 0, ns_ * sizeof(bool));
    }


    void NavFn::setStart(int * start)
    {
        start_[0] = start[0];
        start_[1] = start[1];
    }
    void NavFn::setGoal(int * goal)
    {
        goal_[0] = goal[0];
        goal_[1] = goal[1];   
    }


    void
    NavFn::setCostmap(const COSTTYPE * cmap, bool allow_unknown)
    {
        COSTTYPE * cm = costarr_;
        for (int i = 0; i < ny_; i++) {
            int k = i * nx_;
            for (int j = 0; j < nx_; j++, k++, cmap++, cm++) {
                // This transforms the incoming cost values from NAV2::COSTMAP::2D
                // where in ROS  
                //      NO_INFORMATION = 255;
                //      LETHAL_OBSTACLE = 254;
                //      INSCRIBED_INFLATED_OBSTACLE = 253;
                //      MAX_NON_OBSTACLE = 252;
                //      FREE_SPACE = 0;

                // COST_UNKNOWN_ROS (255)               -> COST_ObS - 1 (253)
                // LETHAL_OBSTACLE (254)                -> COST_OBS (254)
                // INSCRIBED_INFLATED_OBSTACLE (253)    -> COST_OBS (254)
                // values(0 to 252) -> values from COST_NEUTRAL (50) to COST_OBS_ROS (253).
                *cm = COST_OBS;
                int v = *cmap;
                if (v < COST_OBS_ROS) {
                    v = COST_NEUTRAL + COST_FACTOR * v;
                    if (v >= COST_OBS) {
                        v = COST_OBS - 1;
                    }
                    *cm = v;
                } else if (v == COST_UNKNOWN_ROS && allow_unknown) {
                    v = COST_OBS - 1;
                    *cm = v;
                }
            }
        }
    }

    bool NavFn::setNavFn()
    {
        // reset values in propagation arrays
        for (int i = 0; i < ns_; i++) {
            potarr_[i] = POT_HIGHEST;
        }

        // outer bounds of cost array
        COSTTYPE * pc;
        pc = costarr_;
        for (int i = 0; i < nx_; i++) {
            *pc++ = COST_OBS;
        }
        pc = costarr_ + (ny_ - 1) * nx_;
        for (int i = 0; i < nx_; i++) {
            *pc++ = COST_OBS;
        }
        pc = costarr_;
        for (int i = 0; i < ny_; i++, pc += nx_) {
            *pc = COST_OBS;
        }
        pc = costarr_ + nx_ - 1;
        for (int i = 0; i < ny_; i++, pc += nx_) {
            *pc = COST_OBS;
        }

        // priority buffers
        potentThresh_ = COST_OBS;
        n_curPotentArr_ = 0;
        n_nextPotentArr_ = 0;
        n_overflowPotentArr_ = 0;
        memset(pending_, 0, ns_ * sizeof(bool));


        // set start
        int k = start_[0] + start_[1] * nx_;
        potarr_[k] = 0;
        push_cur(k + 1); // right
        push_cur(k - 1); // left
        push_cur(k - nx_); // top
        push_cur(k + nx_); // bottom

        /* TODO TASK - MILESTONE # 1.1
            Check whether top left, top right, bottom left, and bottom right cells
            (not just the right, left, top and bottom)  should be pushed to the 
            buffer "curPotentArr_"
        */
        // Yes: updateCell() uses all 8 neighbours (8-connected grid), so the
        // diagonal neighbours of the start cell must also be expanded first.
        // push_cur() itself rejects invalid indexes, obstacles and pending cells.
        push_cur(k - nx_ - 1); // top-left
        push_cur(k - nx_ + 1); // top-right
        push_cur(k + nx_ - 1); // bottom-left
        push_cur(k + nx_ + 1); // bottom-right


        if (n_curPotentArr_ <= 0)
        {
            RCLCPP_WARN(
                rclcpp::get_logger("rclcpp"),
                "[NavFn] Planning Failed, as Starting cell sorrunded by Obstacles"
            );
            return false;
        }
            

        // find # of obstacle cells
        pc = costarr_;
        int ntot = 0;
        for (int i = 0; i < ns_; i++, pc++) {
            if (*pc >= COST_OBS) {
            ntot++;  // number of cells that are obstacles
            }
        }
        nobs_ = ntot;
        return true;
    }

    bool NavFn::propAstar(int cycles)
    {
        int max_blk_size = 0;  // max priority block size
        int n_cells = 0;  // number of cells put into priority blocks
        int cycle = 0;  // which cycle we're on


        /* TODO TASK - MILESTONE # 2.1
            Compute the heuristic distance from start cell to goal cell
            and add the heuristic distance to the potential threshold
        */
        // Heuristic = straight-line (Euclidean) distance in cells, scaled by COST_NEUTRAL
        // (the cheapest possible cost of moving one cell), so it never over-estimates.
        float dist = std::hypot(goal_[0] - start_[0], goal_[1] - start_[1]) * COST_NEUTRAL;
        // Every cell's priority is (potential + heuristic), which is at least `dist`,
        // so the threshold must start above it or nothing would be "high priority".
        potentThresh_ += dist;

        int * pb;   // pointer used to walk through a buffer
        int i;      // number of cells left to process in that buffer

        bool propSuccess = false;
        for(; cycle < cycles; cycle++)
        {
            /* TODO TASK - MILESTONE # 2.2 
                Check whether the number of valid cell indexes in buffer ``curPotentArr_" and ``nextPotentArr_"
                is equal to zero, if yes, we break the loop
            */
            if (n_curPotentArr_ == 0 && n_nextPotentArr_ == 0)
                break;   // nothing left to expand: goal unreachable (or already done)

            /* TODO TASK - MILESTONE # 2.3
                Set the values of cells in ``pending_" with indexes stored in ``curPotentArr_" to false
            */
            // These cells are about to be processed, so they are no longer "waiting";
            // this lets them be pushed again later if a cheaper route to them is found.
            pb = curPotentArr_;
            i = n_curPotentArr_;
            while (i-- > 0)
            {
                pending_[*(pb++)] = false;
            }

            pb = curPotentArr_;
            i = n_curPotentArr_;
            while(i-- >0)
            {
                updateCell(*pb++);
            }

            //stats
            n_cells += n_curPotentArr_;
            if (n_curPotentArr_>max_blk_size)
                max_blk_size = n_curPotentArr_;

            n_curPotentArr_ = n_nextPotentArr_;
            n_nextPotentArr_ = 0;
            pb =  curPotentArr_;
            curPotentArr_ = nextPotentArr_;
            nextPotentArr_ = pb;

            /* TODO TASK - MILESTONE # 2.4
                Check whether after swapping the contents of buffers ``curPotentArr_" and ``nextPotentArr_"
                there are indexes of cells stored in the ``curPotentArr_". 
                If does not exist any, swap the contents of buffers `curPotentArr_" and ``overflowPotentArr_", 
                and increment the potential threshhold.
            */
            if (n_curPotentArr_ == 0)
            {
                // all high-priority cells are done: promote the low-priority (overflow)
                // cells and raise the threshold so more of them count as high priority
                pb = curPotentArr_;
                curPotentArr_ = overflowPotentArr_;
                overflowPotentArr_ = pb;
                n_curPotentArr_ = n_overflowPotentArr_;
                n_overflowPotentArr_ = 0;
                potentThresh_ += potentThreshInc_;
            }

            /* TODO TASK - MILESTONE # 2.5
                Check whether the propogation hit the goal pose.
                If yes, stop the propogation process and set succes flag to ''true"
            */
            // The goal cell has been given a potential (it was POT_HIGHEST before),
            // i.e. the wavefront has reached it -> a path exists.
            if (potarr_[goal_[1] * nx_ + goal_[0]] < POT_HIGHEST)
            {
                propSuccess = true;
                break;
            }

                
        }
        RCLCPP_DEBUG(
            rclcpp::get_logger("rclcpp"),
            "[NavFn] Used %d cycles, %d cells visited (%d%%), priority buf max %d\n",
            cycle, n_cells, (int)((n_cells * 100.0) / (ns_ - nobs_)), max_blk_size);    
        return propSuccess;
    }

    inline void NavFn::updateCell(int n)
    {
        const double SQUAREROOT2 = 1.41421356237;
        float pot;

        if (costarr_[n] < COST_OBS) // Only update and propagate the cell without obstacle
        {
            float l, r, t, b, tl, tr, bl, br;

            /* TODO TASK - MILESTONE # 3.1
                Compute the potentials moving from the left, right, top, and bottom cells
                and save them to variables l, r, t, and b, respectively. 
                Example, if you move from left cell to the current cell
                l = potarr_[n - 1] + costarr_[n] 
                Please note if you copy code above, an error will appear as it serve for
                the purpose of explanation.
            */
            // cost to reach n = cost to reach the neighbour + cost of stepping into n
            l = potarr_[n - 1]   + costarr_[n];   // from the left cell
            r = potarr_[n + 1]   + costarr_[n];   // from the right cell
            t = potarr_[n - nx_] + costarr_[n];   // from the top cell    (one row up)
            b = potarr_[n + nx_] + costarr_[n];   // from the bottom cell (one row down)


            /* TODO TASK - MILESTONE # 3.2
                Compute the potentials moving from the top-left, top-right, bottom-left, and
                bottom-right cells, and save them to variables tl, tr, bl, and br, respectively.
                The formula used is
                tl = potarr_[n - nx_ - 1] + sqrt(2)*costarr_[n] 
                Please note here, the coefficient sqrt(2) is used because the traversal distance is
                sqrt(2) grid unit.
            */
            // diagonal steps are sqrt(2) times longer, so they cost sqrt(2) times more
            tl = potarr_[n - nx_ - 1] + SQUAREROOT2 * costarr_[n];
            tr = potarr_[n - nx_ + 1] + SQUAREROOT2 * costarr_[n];
            bl = potarr_[n + nx_ - 1] + SQUAREROOT2 * costarr_[n];
            br = potarr_[n + nx_ + 1] + SQUAREROOT2 * costarr_[n];

            /* TODO TASK - MILESTONE # 3.3
                Find the minimum value among l, r, t, b, tl, tr, bl, and br, 
                and set the minimum value as the newly computed potential for cell 
                with index n. 
                The function std::min from the ``algorithm" library can be used. 
                https://en.cppreference.com/w/cpp/algorithm/min
            */
            // best (cheapest) way to reach n from any of its 8 neighbours
            pot = std::min({l, r, t, b, tl, tr, bl, br});


            if (pot < potarr_[n]) /* only update the newly computed pot if it is 
                less than the existing saved potential estimate */ 
            {
                /* TODO TASK - MILESTONE # 3.4
                    Update the potential value of cell with index n
                */
                potarr_[n] = pot;   // store the TRUE cost-so-far g(n) (without heuristic)

                /* TODO TASK - MILESTONE # 3.5
                    Compute the heuristic distance from current cell to the goal cell,
                    and add the distance cost to the pot
                */
                // index n -> cell coordinates (x = column, y = row)
                int x = n % nx_;
                int y = n / nx_;
                float dist = std::hypot(goal_[0] - x, goal_[1] - y) * COST_NEUTRAL;   // h(n)
                pot += dist;        // pot is now the A* priority f(n) = g(n) + h(n)


                if (pot < potentThresh_)
                {
                    /* TODO TASK - MILESTONE # 3.6
                        Check whether current cell's left, right, top, and bottom cells need to update 
                        their potentials. 
                        If the existing saved potential of the neighbouring cell is larger than the 
                        sum of current cell potential,  heuristic distance from current cell to goal cell, 
                        and traversal cost to the neighbouring cell, we need to update the potential of the
                        neighbouring cell. Hence, you have to push the cell to the buffer ``nextPotentArr_".
                        An example, for the left cell of current cell, the condition to be checked is
                            potarr_[n - 1] > pot +  costarr_[n - 1]
                    */
                    // f(n) is below the threshold -> neighbours get HIGH priority (next round)
                    if (potarr_[n - 1]   > pot + costarr_[n - 1])   push_next(n - 1);     // left
                    if (potarr_[n + 1]   > pot + costarr_[n + 1])   push_next(n + 1);     // right
                    if (potarr_[n - nx_] > pot + costarr_[n - nx_]) push_next(n - nx_);   // top
                    if (potarr_[n + nx_] > pot + costarr_[n + nx_]) push_next(n + nx_);   // bottom

                    /* TODO TASK - MILESTONE # 3.7
                        Check whether current cell's top-left, top-right, bottom-left, and bottom-right
                        cells need to update their potentials. 
                        If the existing saved potential of the neighbouring cell is larger than the 
                        sum of current cell potential,  heuristic distance from current cell to goal cell, 
                        and traversal cost to the neighbouring cell, we need to update the potential of the
                        neighbouring cell. Hence, you have to push the cell to the buffer ``nextPotentArr_".
                        An example, for the top-left cell of current cell, the condition to be checked is
                            potarr_[n - nx_ - 1] > pot +  sqrt(2) * costarr_[n - nx_ - 1]
                        Please note the difference in the formula to that of above task.
                    */
                    if (potarr_[n - nx_ - 1] > pot + SQUAREROOT2 * costarr_[n - nx_ - 1]) push_next(n - nx_ - 1); // top-left
                    if (potarr_[n - nx_ + 1] > pot + SQUAREROOT2 * costarr_[n - nx_ + 1]) push_next(n - nx_ + 1); // top-right
                    if (potarr_[n + nx_ - 1] > pot + SQUAREROOT2 * costarr_[n + nx_ - 1]) push_next(n + nx_ - 1); // bottom-left
                    if (potarr_[n + nx_ + 1] > pot + SQUAREROOT2 * costarr_[n + nx_ + 1]) push_next(n + nx_ + 1); // bottom-right

                } else {
                    /* TODO TASK - MILESTONE # 3.8
                        Check whether current cell's left, right, top, and bottom cells need to update 
                        their potentials. 
                        If the existing saved potential of the neighbouring cell is larger than the 
                        sum of current cell potential,  heuristic distance from current cell to goal cell, 
                        and traversal cost to the neighbouring cell, we need to update the potential of the
                        neighbouring cell. Hence, you have to push the cell to the buffer ``overflowPotentArr_".
                        An example, for the left cell of current cell, the condition to be checked is
                            potarr_[n - 1] > pot +  costarr_[n - 1]
                    */
                    // f(n) is above the threshold -> neighbours get LOW priority (overflow)
                    if (potarr_[n - 1]   > pot + costarr_[n - 1])   push_over(n - 1);     // left
                    if (potarr_[n + 1]   > pot + costarr_[n + 1])   push_over(n + 1);     // right
                    if (potarr_[n - nx_] > pot + costarr_[n - nx_]) push_over(n - nx_);   // top
                    if (potarr_[n + nx_] > pot + costarr_[n + nx_]) push_over(n + nx_);   // bottom

                    /* TODO TASK - MILESTONE # 3.9
                        Check whether current cell's top-left, top-right, bottom-left, and bottom-right
                        cells need to update their potentials. 
                        If the existing saved potential of the neighbouring cell is larger than the 
                        sum of current cell potential,  heuristic distance from current cell to goal cell, 
                        and traversal cost to the neighbouring cell, we need to update the potential of the
                        neighbouring cell. Hence, you have to push the cell to the buffer ``overflowPotentArr_".
                        An example, for the top-left cell of current cell, the condition to be checked is
                            potarr_[n - nx_ - 1] > pot + sqrt(2) * costarr_[n - nx_ - 1]
                        Please note the difference in the formula to that of above task.
                    */
                    if (potarr_[n - nx_ - 1] > pot + SQUAREROOT2 * costarr_[n - nx_ - 1]) push_over(n - nx_ - 1); // top-left
                    if (potarr_[n - nx_ + 1] > pot + SQUAREROOT2 * costarr_[n - nx_ + 1]) push_over(n - nx_ + 1); // top-right
                    if (potarr_[n + nx_ - 1] > pot + SQUAREROOT2 * costarr_[n + nx_ - 1]) push_over(n + nx_ - 1); // bottom-left
                    if (potarr_[n + nx_ + 1] > pot + SQUAREROOT2 * costarr_[n + nx_ + 1]) push_over(n + nx_ + 1); // bottom-right

                }
            }
        }

    }

    int NavFn::calcPath(int n)
    {
        if (npathbuf_ < n) {
            if (pathx_) {delete[] pathx_;}
            if (pathy_) {delete[] pathy_;}
            pathx_ = new float[n];
            pathy_ = new float[n];
            npathbuf_ = n;
        }
        int * st;
        st = goal_;
        int stc = st[1] * nx_ + st[0];
        npath_ = 0;

        for(int i = 0; i < n; i++)
        {
            pathx_[npath_] = stc % nx_;
            pathy_[npath_] = stc / nx_;
            npath_++;
            
            if (potarr_[stc] < COST_NEUTRAL) {
                return npath_;  // done!
            }

            int stcnx = stc + nx_;
            int stcpx = stc - nx_;

            int minc = stc;
            int curp = potarr_[stc];
            int minp = curp;

            /* TODO TASK - MILESTONE # 4.1
                Among eight neighbouring cells of current cell (with index as ``stc")
                find a cell with minimum potential, and save its index to stc, and its
                potential to ''minp"
            */
            // 8 neighbours: left, right, top, bottom, top-left, top-right, bottom-left, bottom-right
            const int nbrs[8] = {stc - 1, stc + 1, stcpx, stcnx,
                                 stcpx - 1, stcpx + 1, stcnx - 1, stcnx + 1};
            for (int k = 0; k < 8; k++)
            {
                const int nb = nbrs[k];
                if (nb < 0 || nb >= ns_)
                    continue;                       // outside the map
                if (potarr_[nb] < minp)             // lower potential = closer to the start
                {
                    minp = potarr_[nb];
                    minc = nb;
                }
            }
            stc = minc;   // step to the best neighbour (next point of the path)

            if(minp >= curp)
            {
                RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] Zero gradient");
                return 0;
            }
        }

        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] No path found, path too long");
        return 0;
    }
    
}