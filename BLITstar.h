/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, University of California, Irvine
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the names of the copyright holders nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Authors: Yi Wang

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BLITSTAR_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BLITSTAR_

#include <algorithm>
#include <memory>
#include <unordered_map>
#include "ompl/base/Planner.h"
#include <unsupported/Eigen/Polynomials>
#include "ompl/geometric/PathGeometric.h"
#include "ompl/geometric/planners/lazyinformedtrees/blitstar/ImplicitGraph.h"
#include "ompl/geometric/planners/lazyinformedtrees/blitstar/Vertex.h"
#include "ompl/geometric/planners/lazyinformedtrees/blitstar/Queuetypes.h"

//#include "ompl/geometric/planners/LQR.h"
//#include "ompl/geometric/planners/DuR.h"
#include "ompl/geometric/planners/DIR.h"
#include <chrono>
using namespace std::chrono;
namespace ompl
{
    namespace geometric
    {
        /**
        @anchor gbLitstar

        \ref blitstar "BLIT*" (Bidirectional Lazy Informed Trees) is an almost-surely asymptotically optimal motion planner.
                       This version extends BLIT* with time-parameterized collision detection for dynamic environments. 
   
        */

        class BLITstar : public ompl::base::Planner
        {
        public:
            /** \brief Short names for commonly used BLIT* vertex types. */
            using VertexPtr = blitstar::VertexPtr;
            using KeyVertexPair = blitstar::KeyVertexPair; 
            using Cost = ompl::base::Cost;  
                      
            /** \brief Constructs a BLIT*. */
            explicit BLITstar(const ompl::base::SpaceInformationPtr &spaceInformation);

            /** \brief Destructs a BLIT*. */
            ~BLITstar();

            /** \brief Additional setup that can only be done once a problem definition is set. */
            void setup() override;
            /** \brief time to find a solution.*/
            void runTime();     
            /** \brief Checks whether the planner is successfully setup. */
            ompl::base::PlannerStatus::StatusType ensureSetup();

            /** \brief Checks whether the problem is successfully setup. */
            ompl::base::PlannerStatus::StatusType
            ensureStartAndGoalStates(const ompl::base::PlannerTerminationCondition &terminationCondition);

            /** \brief Clears the algorithm's internal state. */
            void clear() override;

            /** \brief Solves a motion planning problem. */
            ompl::base::PlannerStatus
            solve(const ompl::base::PlannerTerminationCondition &terminationCondition) override;
            
            /** \brief Get the planner data. */
            void getPlannerData(base::PlannerData &data) const override;
	    
            /** \brief Set the batch size. */
            void setBatchSize(std::size_t batchSize);

            /** \brief Get the batch size. */
            std::size_t getBatchSize() const;

            /** \brief Set the rewire factor of the RGG graph. */
            void setRewireFactor(double rewireFactor);
             
            /** \brief Get the rewire factor of the RGG graph. */
            double getRewireFactor() const;
            
            std::vector<double> timeSlice(double time_, int num); 
            /** \brief Set whether to track approximate solutions. */
            
            void trackApproximateSolutions(bool track);

            /** \brief Get whether approximate solutions are tracked. */
            bool areApproximateSolutionsTracked() const;

            /** \brief Set whether pruning is enabled or not. */
            void enablePruning(bool prune);

            /** \brief Get whether pruning is enabled or not. */
            bool isPruningEnabled() const;

            /** \brief Set whether to use a k-nearest RGG connection model. */
            void setUseKNearest(bool useKNearest);

            /** \brief Get whether to use a k-nearest RGG connection model. */
            bool getUseKNearest() const;

            /** \brief Set the maximum number of goals BLIT* will sample from sampleable goal regions. */
            void setMaxNumberOfGoals(unsigned int numberOfGoals);

            /** \brief Get the maximum number of goals BLIT* will sample from sampleable goal regions. */
            unsigned int getMaxNumberOfGoals() const;
            
            /** \brief Perform compelete collision detection. */
            bool CCD(const blitstar::keyEdgePair &edge, double arriveTime_);
            
            /** \brief Perform sparse collision detection. */
            bool SCD(const blitstar::keyEdgePair &edge, double arriveTime_);
       
            /** \brief Clears the reverse or forward vertex queue. */
            void clearReverseVertexQueue();
            void clearForwardVertexQueue();
            
            /** \brief Resets the reverse or forward parent and g-cost of the vertex. */
            void resetReverseValue(const VertexPtr &vertex);
            void resetForwardValue(const VertexPtr &vertex); 
            
            /** \brief Terminates the search while ensuring the meet-in-the-middle property. */
            bool terminateSearch(); 
            
            /** \brief Inserts the start and goal into the forward or reverse queue. */
            void insertGoalInReverseQueue();  
            void insertStartInForwardQueue();
            
            /** \brief Labels both endpoints of an edge that lies near an obstacle. */
            void markNearObstacle(const VertexPtr &child, const VertexPtr &parent, bool forward_);
            void markInCollision(const VertexPtr &child, const VertexPtr &parent, bool forward_, bool detach_);
            
            /** \brief Resets the forward or reverse parent information of the vertex. */
            void resetForwardParentInformation(const VertexPtr & vertex);   
            void resetReverseParentInformation(const VertexPtr & vertex); 
            
            /** \brief Selects the vertex with minimum priority in both queues for expansion. */
            bool selectExpandState(bool & forward);      
            
            /** \brief Checks whether the candidate path is collision-free. */
            bool pathValidity(VertexPtr &vertex); 
            void forwardPathValidating(VertexPtr &vertex, bool &validity);
            void reversePathValidating(VertexPtr &vertex, bool &validity);
            
            /** \brief Prints debugging information for the forward or reverse search. */
            void printForwardStatement(const VertexPtr &vertex);
            void printReverseStatement(const VertexPtr &vertex);

            /** \brief Finds the minimal f-value among all children of the vertex. */             
            void findBestNeighbor(Cost curMin_, size_t neighbor); 
            void bestNeighbor(Cost costToCome, Cost costToGoal, size_t neighbor);
            
            /** \brief Searches in each search direction. */ 
            void forwardLazySearch(const VertexPtr &vertex);
            void reverseLazySearch(const VertexPtr &vertex);
            
            /** \brief Checks whether the promissing edge is potentially collision-free. */ 
            bool isEdgeValid(const blitstar::keyEdgePair &edge, std::size_t numChecks, bool sparseCheck, double arriveTime_);
            
            /** \brief Re-inserts a vertex into each search direction. */ 
            void reInsertVertexInReverseQueue(const VertexPtr &vertex, Cost costToCome, Cost &costToGo);
            void reInsertVertexInForwardQueue(const VertexPtr &vertex, Cost costToCome, Cost &costToGo);
            
            /** \brief Inserts a vertex into each search direction. */ 
            void insertVertexInForwardQueue(const VertexPtr &vertex, ob::Cost gCost_, ob::Cost hCost_, ob::Cost curCost_, bool meet_);
            void insertVertexInReverseQueue(const VertexPtr &vertex, ob::Cost gCost_, ob::Cost hCost_, ob::Cost curCost_, bool meet_);
            
            /** \brief Checks whether this vertex provides a better solution. If so, mark it. */                    
            void checkBetterSolution(const VertexPtr &vertex, bool intersected, bool &meet_, Cost &gCur_, Cost gCost_);
            void markMeetVertex(const VertexPtr &vertex, Cost meetCost); 

            /** \brief Checks whether the start and goal are too close to each other. */       
            void rootsCollision(const VertexPtr &start, const VertexPtr &goal);
            
            /** \brief Inserts or updates a vertex in the reverse or forward queue. */
            void forwardVertexQueue(const VertexPtr &vertex, Cost CostToCome, Cost CostToGoal);
            void reverseVertexQueue(const VertexPtr &vertex, Cost CostToCome, Cost CostToGoal);
            
            /** \brief Returns the zero cost. */
            ompl::base::Cost zeroCost() const;            
            
            /** \brief Checks the start-goal edge before the search and registers it as the solution if collision-free. */
            bool foundTrivialSolution();
            
            /** \brief Returns the motion cost of an edge computed using the robot model. */
            ob::Cost robotMotionCost(const VertexPtr &target, const VertexPtr &source);
            
            /** \brief Refines the h-cost this vertex if needed. */
            void refineHCost(const VertexPtr &vertex, Cost &costToCome, Cost &costToGo, Cost gCur_,bool meet_);  
            
            /** \brief Compares or sum two costs. */
            bool betterCost(const Cost & cost1, const Cost & cost2);
            bool largerCost(const Cost & cost1, const Cost & cost2);
            ompl::base::Cost sumCost(const Cost & cost1, const Cost & cost2);

        private:
            /** \brief Performs one iteration of BLIT*. */
            void iterate(const ompl::base::PlannerTerminationCondition &terminationCondition);
            
            /** \brief Emit one NDJSON visualization frame (instrumentation). */
            void logFrame(const std::string &note, const VertexPtr &a, const VertexPtr &b, const char *status);
            
            /** \brief The space information of the planning problem. */
            ompl::base::SpaceInformationPtr spaceInformation_;

            /** \brief An interpolated state for collision checks. */
            ompl::base::State *detectionState_; 
            
            /** \brief The option that specifies whether to track approximate solutions. */
            bool trackApproximateSolutions_{true};
            
            /** \brief The option that specifies whether to prune the graph of useless samples. */
            bool isPruningEnabled_{false};
            
            /** \brief Prints a message using OMPL_INFORM to let the user know that AIT* found a new solution. */
            void informAboutNewSolution() const;
            
            /** \brief Prints a message using OMPL_INFORM to let the user know of the planner status. */
            void informAboutPlannerStatus(ompl::base::PlannerStatus::StatusType status) const;

            /** \brief Updates the exact solution and if BLIT* track approximate solutions.*/
            ompl::base::PlannerStatus::StatusType updateSolution();

            /** \brief Updates the exact solution and if BLIT* track approximate solutions. */
            ompl::base::PlannerStatus::StatusType updateSolution(const VertexPtr &vertex);

            /** \brief Counts the number of vertices in the forward tree. */
            std::size_t countNumVerticesInForwardTree() const;

            /** \brief Counts the number of vertices in the reverse tree. */
            std::size_t countNumVerticesInReverseTree() const;

            /** \brief Returns the path a start to the argument. */
            std::shared_ptr<ompl::geometric::PathGeometric> getPathToVertex(const VertexPtr &vertex) const;

            /** \brief Returns the estimated f-cost given the costs from the start to the goal going through this vertex. */
            std::array<Cost, 2u> estimatedPathCosts(Cost CostToStart, Cost CostToGoal) const;

            /** \brief Lexicographically compares the keys of two vertices. */
            bool isVertexBetter(const blitstar::KeyVertexPair &lhs, const blitstar::KeyVertexPair &rhs) const;

            /** \brief Updates the current best solution. */
            void updateExactSolution();

            /** \brief The cost of the incumbent solution. */
            Cost solutionCost_;

            /** \brief The optimal solution found so far for a give RGG. */          
            Cost C_curr;
            
            /** \brief The cost to come to the vertex that is closest to the goal (in cost space). */
            Cost approximateSolutionCost_{};

            /** \brief The cost to go to the goal from the current best approximate solution. */
            Cost approximateSolutionCostToGoal_{};
            
            /** \brief The minimal f-values across both queues. */
            Cost fmin_{0u};
            
            /** \brief The minimal f-value of the forward queue. */
            Cost forwardCost{0u};
            
            /** \brief The minimal f-value of the reverse queue. */
            Cost reverseCost{0u}; 
            
            /** \brief The minimal f-values among all children of this vertex. */
            Cost minimalneighbor_{0u};
            
            /** \brief Whether the start and goal are too close to each other. */
            bool rootsCollided_{false};
            
            /** \brief The increasingly dense sampling-based approximation. */
            blitstar::ImplicitGraph graph_;

            /** \brief The forward or reverse vertex queue. */
            blitstar::VertexQueue forwardVertexQueue_;
            blitstar::VertexQueue reverseVertexQueue_;

            
            // std::vector<std::pair<VertexPtr, VertexPtr>> FLVEdge;
            // std::vector<std::pair<VertexPtr, VertexPtr>> IFLVEdge;
            // std::vector<std::pair<VertexPtr, VertexPtr>> RLVEdge;
            // std::vector<std::pair<VertexPtr, VertexPtr>> IRLVEdge;
            
            
            // std::vector<std::pair<VertexPtr, VertexPtr>> FVEdge;
            // std::vector<std::pair<VertexPtr, VertexPtr>> IFVEdge;
            // std::vector<std::pair<VertexPtr, VertexPtr>> RVEdge;
            // std::vector<std::pair<VertexPtr, VertexPtr>> IRVEdge;
            
            /** \biref the best vertex*/
            VertexPtr bestVertex_;
            
            /** \brief The current best solution path. */
            std::shared_ptr<ompl::geometric::PathGeometric> path_;
            
            /** \biref The meeting vertex*/
            blitstar::MiddleVertex V_meet;  

            /** \brief The number of iterations that have been performed. */
            std::size_t numIterations_{0u};
            
            /** \brief The maximum collision-detection resolution needed so far. */
            std::size_t maxSparseResolution_{1u};
            
            /** \brief The id of the current forward or reverse search. */
            std::size_t forwardId_{0u};
            std::size_t reverseId_{0u};
            
            /** \brief The number of samples per batch. */
            std::size_t batchSize_{300u};
            
            /** \brief Whether a new meeting vertex is found. */
            bool meetVertex_{false};
            
            /** \brief Whether the search must restart from scratch. */
            bool start_scratch_{false};
            void showingRunningtime();
            
            /** \brief Whether cost1 is smaller than cost2. */
            bool betterThan(const Cost & cost1, const Cost & cost2);
            
            /** \brief The current search direction (true = forward). */
            bool searchDir_{false};
            
            /** \brief Whether an initial solution has been found. */
            bool iSolution_{false};         
            
            /** \brief Adding more samples. */
            bool isVertexEmpty_{true};
            
            /** \brief Finding a solution. */
            bool found_meeting_{false};
            
            /** \brief An invalid search direction. */
            bool forwardInvalid_{false};
            bool reverseInvalid_{false};  
            
            /** \brief Whether a better solution was found within this RGG. */
            bool betterSolution_{false};
            
            /** \brief Whether planning for a real robot model. */
            bool realRobotModel_{false};
            
            double optTrajTime_{0u};
            
            double time_taken{0u};
            
            high_resolution_clock::time_point  startT, endT;
            
            /** \brief Syntactic helper to get at the optimization objective of the planner base class. */
            ompl::base::OptimizationObjectivePtr objective_;

            /** \brief Syntactic helper to get at the motion validator of the planner base class. */
            ompl::base::MotionValidatorPtr motionValidator_;

            /** \brief The number of processed edges. */
            std::size_t numProcessedEdges_{0u};
            
            /** \brief The number of processed batches. */
            std::size_t numbatch_{0u};
            
            /** \brief The state space of the planning problem. */
            std::shared_ptr<ompl::base::StateSpace> space_;
            
            /** \brief The number of edge collision checks performed. */
            std::size_t numEdgeCollisionChecks_{0u};
        };
    }  // namespace geometric
}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BLITSTAR
