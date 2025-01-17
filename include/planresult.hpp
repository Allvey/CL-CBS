/**
 * @file planresult.hpp
 * @author Licheng Wen (wenlc@zju.edu.cn)
 * @brief PlanResult header
 * @date 2020-11-12
 *
 * @copyright Copyright (c) 2020
 *
 */
#pragma once

#include <vector>

namespace libMultiRobotPlanning {

/*! \brief Represents the path for an agent

		This class is used to store the result of a planner for a single agent.
		It has both the ordered list of states that need to be traversed as well as
		the ordered list of actions together with their respective costs

		\tparam State Custom state for the search. Needs to be copy'able
		\tparam Action Custom action for the search. Needs to be copy'able
		\tparam Cost Custom Cost type (integer or floating point types)
*/
template <typename State, typename Action, typename Cost>
struct PlanResult {
	//! states and their gScore
	std::vector<std::pair<State, Cost>> states;
	//! actions and their cost
	std::vector<std::pair<Action, Cost>> actions;
	//! states and their collision score - zxt
	std::vector<std::pair<State, Cost>> collisions;
	//! actual cost of the result
	Cost pcost;
	//! lower bound of the cost (for suboptimal solvers)
	Cost fmin;
	//！ collision cost between agents - zxt
	Cost ccost;
};

}  // namespace libMultiRobotPlanning
