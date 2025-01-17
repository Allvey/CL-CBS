/**
 * @file neighbor.hpp
 * @author Licheng Wen (wenlc@zju.edu.cn)
 * @brief Neighbor header
 * @date 2020-11-12
 *
 * @copyright Copyright (c) 2020
 *
 */
#pragma once

namespace libMultiRobotPlanning {

/*! \brief Represents state transations

    This class represents a transition from a start state applying an action
   with the given cost.

    \tparam State Custom state for the search. Needs to be copy'able
    \tparam Action Custom action for the search. Needs to be copy'able
    \tparam Cost Custom Cost type (integer or floating point types)
*/
template <typename State, typename Action, typename Cost>
struct Neighbor {
  Neighbor(const State& state, const Action& action, Cost cost, Cost collision)
      : state(state), action(action), cost(cost), collision(collision) {}

  //! neighboring state
  State state;
  //! action to get to the neighboring state
  Action action;
  //! cost to get to the neighboring state
  Cost cost;
  //! cost to get to the neighboring state - zxt
  Cost collision;
};

}  // namespace libMultiRobotPlanning
