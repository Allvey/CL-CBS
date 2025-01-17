/**
 * @file hybrid_astar.hpp
 * @author Licheng Wen (wenlc@zju.edu.cn)
 * @brief hybrid Astar header
 * @date 2020-11-12
 *
 * @copyright Copyright (c) 2020
 *
 */
#pragma once
#include <algorithm>
#include <cmath>
#include <iostream>
#include <unordered_map>
#include <unordered_set>

#define USE_FIBONACCI_HEAP
#include <boost/heap/d_ary_heap.hpp>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/program_options.hpp>

#include "neighbor.hpp"
#include "planresult.hpp"

namespace libMultiRobotPlanning {
/*! 
\example sh_astar.cpp Implementation of Spatiotemporal Hybrid-State A*
algorithm.
*/

/*! \brief Spatiotemporal Hybrid A* Algorithm.

This class implements the Spatiotemporal Hybrid-State A* algorithm. **Noted
that it could also used as traditional Hybrid-State Astar using correct
implementation**

This class can either use a fibonacci heap, or a d-ary heap. The former is the
default. Comment "USE_FIBONACCI_HEAP" to use the d-ary heap instead.

\tparam State Custom state for the search. Needs to be copy'able
\tparam Action Custom action for the search. Needs to be copy'able
\tparam Cost Custom Cost type (integer or floating point types)
\tparam Environment This class needs to provide the custom hybrid A* logic. In
		particular, it needs to support the following functions:
	- `Cost admissibleHeuristic(const State& s)`\n
		This function can return 0 if no suitable heuristic is available.

	- `bool isSolution(const State& s, Cost gscore,
			std::unordered_map<State, std::tuple<State, Action, Cost, Cost>`,
												StateHasher>& cameFrom)\n
		 Return true if the given state is close to goal state and it has a
		collision-free path to goal. It is also needed to insert the generate
		path-to-goal into the current cameFrom map.

	- `State getGoal()`\n
		Return goal state of current agent.

	- `void getNeighbors(const State& s, std::vector<Neighbor<State, Action,
	 Cost> >& neighbors)`\n
		Fill the list of neighboring state for the given state s.

	- `int calcIndex(const State& s)`\n
		This function calculate a index(int) for a given state s, the index uses as
		a key in closed list.

	- `void onExpandNode(const State& s, Cost fScore, Cost gScore)`\n
		This function is called on every expansion and can be used for statistical
		purposes.

	- `void onDiscover(const State& s, Cost fScore, Cost gScore)`\n
		This function is called on every node discovery and can be used for
	 statistical purposes.

\tparam StateHasher A class to convert a state to a hash value. Default:
	 std::hash<State>
*/

template <typename State, typename Action, typename Cost, typename Environment,
					typename StateHasher = std::hash<State>>
class HybridAStar {
 public:
	HybridAStar(Environment& environment) : m_env(environment) {}
	~HybridAStar() {}

	bool search(const State& startState, PlanResult<State, Action, Cost>& solution, int hardCheck, 
			Cost initialCost = 0, Cost freeCollision = 0) {
		solution.states.clear();
		solution.actions.clear();
		solution.pcost = 0;
		solution.ccost = 0;

		openSet_t openSet;
		std::unordered_map<uint64_t, heapHandle_t, std::hash<uint64_t>> stateToHeap;
		std::unordered_set<uint64_t, std::hash<uint64_t>> closedSet;
		std::unordered_map<State, std::tuple<State, Action, Cost, Cost>, StateHasher> cameFrom;
		
		// Node(State, Action, fScore, gScore, cScore))
		auto handle = openSet.push(Node(startState, Action(), m_env.admissibleHeuristic(startState), initialCost, freeCollision));
		stateToHeap.insert(std::make_pair<>(m_env.calcIndex(startState), handle));
		(*handle).handle = handle;
		
		// 定义邻域状态
		std::vector<Neighbor<State, Action, Cost>> neighbors;
		neighbors.reserve(10);

		while (!openSet.empty()) {
			Node current = openSet.top();
			
			// lowlevelNode 数量加一
			m_env.onExpandNode(current.state, current.fScore, current.gScore);

			// 若 one shot 成功
			if (m_env.isSolution(current.state, current.gScore, cameFrom)) {
				solution.states.clear();
				solution.actions.clear();

				auto iter = cameFrom.find(m_env.getGoal());

				// std::unordered_map<State, std::tuple<State, Action, double /*action cost*/, double /*path cost (gScore)*/>, std::hash<State>>
				solution.pcost = std::get<3>(iter->second);
				solution.fmin =
						std::get<3>(iter->second) +
						m_env.admissibleHeuristic(iter->first);  // current.fScore;
				
				// TODO: 因为此处没有进行碰撞检测，暂时设为0
				solution.ccost = 0;
				 
				while (iter != cameFrom.end()) {
					// std::cout << " From " << std::get<0>(iter->second)
					//           << " to Node:" << iter->first
					//           << " with ACTION: " << std::get<1>(iter->second) << "
					//           cost "
					//           << std::get<2>(iter->second) << " g_score "
					//           << std::get<3>(iter->second) << std::endl;
					solution.states.push_back(
							std::make_pair<>(iter->first, std::get<3>(iter->second)));
					solution.actions.push_back(std::make_pair<>(
							std::get<1>(iter->second), std::get<2>(iter->second)));
					iter = cameFrom.find(std::get<0>(iter->second));
				}
				solution.states.push_back(std::make_pair<>(startState, initialCost));
				std::reverse(solution.states.begin(), solution.states.end());
				std::reverse(solution.actions.begin(), solution.actions.end());

				openSet.clear();
				return true;
			}

			// one shot 失败，进行混合 A* 搜索
			openSet.pop();
			stateToHeap.erase(m_env.calcIndex(current.state));
			closedSet.insert(m_env.calcIndex(current.state));
			
			// 获取邻居节点
			neighbors.clear();
			m_env.getNeighbors(current.state, current.action, neighbors, hardCheck);

			// 遍历邻居节点
			for (const Neighbor<State, Action, Cost>& neighbor : neighbors) {
				if (closedSet.find(m_env.calcIndex(neighbor.state)) ==
						closedSet.end()) {  // 邻居节点不在 closed list

					// 计算当前邻居节点的 cScore
					double cScore = current.cScore + neighbor.collision;

					// 计算当前邻居节点的 gScore
					Cost tentative_gScore = current.gScore + neighbor.cost; // state cost + action cost + collision cost

					// 在 open list 中寻找（stateToHeap）
					auto iter = stateToHeap.find(m_env.calcIndex(neighbor.state));
					
					if (iter == stateToHeap.end()) {  // 邻居节点不在 open list，发现一个新的节点
						
						// 计算当前邻居节点的 fScore
						Cost fScore = tentative_gScore + m_env.admissibleHeuristic(neighbor.state);

						// 加入 open list
						auto handle = openSet.push(Node(neighbor.state, neighbor.action, fScore, tentative_gScore, freeCollision));
						(*handle).handle = handle;
						stateToHeap.insert(std::make_pair<>(m_env.calcIndex(neighbor.state), handle));
						
						// 该函数弃用
						m_env.onDiscover(neighbor.state, fScore, tentative_gScore);
						// std::cout << "  this is a new node " << fScore << "," <<
						// tentative_gScore << std::endl;
					} else {
						auto handle = iter->second;
						// std::cout << "  this is an old node: " << tentative_gScore << ","
						// << (*handle).gScore << std::endl;
						// We found this node before with a better path
						if (tentative_gScore >= (*handle).gScore) {
							continue;
						}

						// 若通过当前节点访问邻居节点的代价更小，则更新
						Cost delta = (*handle).gScore - tentative_gScore;

						(*handle).gScore = tentative_gScore;
						(*handle).fScore -= delta;
						(*handle).state = neighbor.state;
						openSet.increase(handle);
						m_env.onDiscover(neighbor.state, (*handle).fScore, (*handle).gScore);
					}

					// Best path for this node so far
					// TODO: this is not the best way to update "cameFrom", but otherwise
					// default c'tors of State and Action are required
					cameFrom.erase(neighbor.state);
					cameFrom.insert(std::make_pair<>(
							neighbor.state,
							std::make_tuple<>(current.state, neighbor.action, neighbor.cost, tentative_gScore)));
				}
			}
		}
		openSet.clear();
		return false;
	}

 private:
	struct Node {
		Node(const State& state, Action action, Cost fScore, Cost gScore, Cost cScore)
				: state(state), action(action), fScore(fScore), gScore(gScore), cScore{cScore} {}

		bool operator<(const Node& other) const {
			// Sort order
			// 1. lowest fScore
			// 2. highest gScore

			// Our heap is a maximum heap, so we invert the comperator function here
			if (fScore != other.fScore) {
				return fScore > other.fScore;
			} else {
				return gScore < other.gScore;
			}
		}

		friend std::ostream& operator<<(std::ostream& os, const Node& node) {
			os << "state: " << node.state << " fScore: " << node.fScore
				 << " gScore: " << node.gScore;
			return os;
		}

		State state;
		Action action;
		Cost fScore;
		Cost gScore;
		Cost cScore;

#ifdef USE_FIBONACCI_HEAP
		typename boost::heap::fibonacci_heap<Node>::handle_type handle;
#else
		typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>, 
				boost::heap::mutable_<true>>::handle_type
				handle;
#endif
	};

#ifdef USE_FIBONACCI_HEAP
	typedef typename boost::heap::fibonacci_heap<Node> openSet_t;
	typedef typename openSet_t::handle_type heapHandle_t;
#else
	typedef typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>,
																					 boost::heap::mutable_<true>>
			openSet_t;
	typedef typename openSet_t::handle_type heapHandle_t;
#endif

 private:
	Environment& m_env;
};

}  // namespace libMultiRobotPlanning