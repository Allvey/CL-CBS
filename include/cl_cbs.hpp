/**
 * @file cl_cbs.hpp
 * @author Licheng Wen (wenlc@zju.edu.cn)
 * @brief CL-CBS header
 * @date 2020-11-12
 *
 * @copyright Copyright (c) 2020
 *
 */
#pragma once

#include <chrono>
#include <map>
#include <algorithm>

#include "hybrid_astar.hpp"

#define MAX_RUNTIME 600

namespace libMultiRobotPlanning {
/*!
\example cl_cbs.cpp Implementation of whole CL-CBS method
*/

/*! \brief Car-Like Conflict-Based Search(CL-CBS) algorithm to solve the
Multi-Agent Path Finding for Car-Like robots (CL-MAPF) problem

It applies a body conflict tree to address collisions considering shapes of
the agents. It uses a new algorithm called Spatiotemporal Hybrid-State A* as the
single-agent path planner to generate path satisfying both kinematic and
spatiotemporal constraints. The file also integrates a sequential planning
version of CL-CBS method for the sake of efficiency.

Details of the algorithm can be found in the
following paper:\n
Licheng Wen, Zhen Zhang, Zhe Chen, Xiangrui Zhao, and Yong Liu. CL-MAPF:
Multi-Agent Path Finding for Car-Like Robots with Kinematic and Spatiotemporal
Constraints.[[arxiv](https://arxiv.org/abs/2011.00441)]



\tparam State Custom state for the search. Needs to be copy'able
\tparam Action Custom action for the search. Needs to be copy'able
\tparam Cost Custom Cost type (integer or floating point types)
\tparam Conflict Custom conflict description. A conflict needs to be able to be
transformed into a constraint.
\tparam Constraints Custom constraint description. The Environment needs to be
able to search on the low-level while taking the constraints into account.
\tparam Environment This class needs to provide the custom logic. In particular,
it needs to support the following functions:
	- `void setLowLevelContext(size_t agentIdx, const Constraints* constraints)`\n
		Set the current context to a particular agent with the given set of
constraints

	- `Cost admissibleHeuristic(const State& s)`\n
		Admissible heuristic. Needs to take current context into account.

	- `bool isSolution(const State& s, Cost gscore,
			std::unordered_map<State, std::tuple<State, Action, Cost, Cost>`,
												StateHasher>& cameFrom)\n
		 Return true if the given state is a goal state for the current agent. Add
analatic expansion into camefrom struct.

	- `void getNeighbors(const State& s, std::vector<Neighbor<State, Action, int>
>& neighbors)`\n
		Fill the list of neighboring state for the given state s and the current
agent.

	- `bool getFirstConflict(const std::vector<PlanResult<State, Action, int> >&
solution, Conflict& result)`\n
		Finds the first body conflict for the given solution for each agent. Return
true if a conflict conflict was found and false otherwise.

	- `void createConstraintsFromConflict(const Conflict& conflict,
std::map<size_t, Constraints>& constraints)`\n
		Create a list of constraints for the given conflict.

	- `void onExpandHighLevelNode(Cost cost)`\n
		This function is called on every high-level expansion and can be used for
statistical purposes.

	- `void onExpandLowLevelNode(const State& s, Cost fScore, Cost gScore)`\n
		This function is called on every low-level expansion and can be used for
statistical purposes.
*/

template <typename State, typename Action, typename Cost, typename Conflict,
					typename Constraints, typename Constraint, typename Environment>
class CL_CBS {
 public:
	CL_CBS(Environment& environment) : m_env(environment) {}

	~CL_CBS() {}

	bool search(const std::vector<State>& initialStates, std::vector<PlanResult<State, Action, Cost>>& solution) {
		HighLevelNode start;

		start.solution.resize(initialStates.size());
		start.constraints.resize(initialStates.size());

		start.cost = 0;
		start.id = 0;

		// 安全圈缩放比例
		double scale = 5;

		/*
		  使用混合A*赋予所有车辆初始轨迹
		*/
		for (size_t i = 0; i < initialStates.size(); ++i) {
			// if (i < solution.size() && solution[i].states.size() > 1) {
			//   start.solution[i] = solution[i];
			//   std::cout << "use existing solution for agent: " << i << std::endl;
			// } else {
			LowLevelEnvironment llenv(m_env, i, start.constraints[i], 1);
			LowLevelSearch_t lowLevel(llenv);

			// 对于智能体 i 在 lowlevel 进行混合 A* 搜索
			bool success = lowLevel.search(initialStates[i], start.solution[i], 1);
			if (!success) {
				return false;
			}

			// 将单智能体代价累加到多多智能体路径代价上
			start.cost += start.solution[i].pcost;
		}

		/*
		  HighLevel Search
		*/

		// 设置 HighLevel 堆栈
		// std::priority_queue<HighLevelNode> open;
		typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>, boost::heap::mutable_<true>>
				open;

		auto handle = open.push(start);
		(*handle).handle = handle;

		// 设置计时器
		std::chrono::high_resolution_clock::time_point
				startTime = std::chrono::high_resolution_clock::now(),
				endTime;

		solution.clear();
		int id = 1;

		while (!open.empty()) {


			// 超时判断
			endTime = std::chrono::high_resolution_clock::now();
			if (std::chrono::duration_cast<std::chrono::duration<double>>(endTime -
																																		startTime)
							.count() > MAX_RUNTIME) {
				open.clear();
				std::cout << "\033[1m\033[31m Plan out of runtime time! \033[0m\n";
				return false;
			}

			// 取出栈顶节点
			HighLevelNode P = open.top();
			m_env.onExpandHighLevelNode(P.cost);
			// std::cout << "expand: " << P << std::endl;

			// 检测首个冲突，用于生成新节点
			Conflict conflict;
			if (!m_env.getFirstConflict(P.solution, conflict, 1)) {
				// // 如果比例恢复到正常值
				// if (scale <= 1) {
				// 	// std::cout << "done; cost: " << P.cost << std::endl;
				// 	solution = P.solution;
				// 	return true;
				// } else { // 否则放大比例，继续优化
				// 	// scale -= 1;
				// 	m_env.setLowLevelScale(std::max(scale, (double)1));
				// 	continue;
				// }

				solution = P.solution;
				return true;
			}

			// 构造新的冲突树节点
			HighLevelNode newNode = P;
			newNode.id = id;

			newNode.cost -= newNode.solution[conflict.agent1].pcost;

			newNode.cost -= newNode.solution[conflict.agent2].pcost;

			// std::cout << conflict.time << std::endl;

			// 构造耦合的 low level 环境
			LowLevelDuelEnvironment lldenv(m_env, conflict, initialStates, scale);

			bool success = lldenv.solve(newNode.solution);

			newNode.cost += newNode.solution[conflict.agent1].pcost;

			newNode.cost += newNode.solution[conflict.agent2].pcost;

			if (success) {
				open.pop();
				// std::cout << "  success. cost: " << newNode.cost << std::endl;
				auto handle = open.push(newNode);
				(*handle).handle = handle;
			}

			id++;

			// scale -= 1;
			// m_env.setLowLevelScale(std::max(scale, (double)1));

		}
		return false;
	}

 private:
	struct HighLevelNode {
		std::vector<PlanResult<State, Action, Cost>> solution;
		std::vector<Constraints> constraints;

		Cost cost;

		int id;

		typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>, boost::heap::mutable_<true>>::handle_type
				handle;

		bool operator<(const HighLevelNode& n) const {
			// if (cost != n.cost)
			return cost > n.cost;
			// return id > n.id;
		}

		friend std::ostream& operator<<(std::ostream& os, const HighLevelNode& c) {
			os << "id: " << c.id << " cost: " << c.cost << std::endl;
			for (size_t i = 0; i < c.solution.size(); ++i) {
				os << "Agent: " << i << std::endl;
				os << " States:" << std::endl;
				for (size_t t = 0; t < c.solution[i].states.size(); ++t) {
					os << "  " << c.solution[i].states[t].first << std::endl;
				}
				os << " Constraints:" << std::endl;
				os << c.constraints[i];
				os << " cost: " << c.solution[i].cost << std::endl;
			}
			return os;
		}
	};

	struct LowLevelEnvironment {
		LowLevelEnvironment(Environment& env, size_t agentIdx, const Constraints& constraints, double scale = 1)
				: m_env(env), scale(scale)
		// , m_agentIdx(agentIdx)
		// , m_constraints(constraints)
		{
			m_env.setLowLevelContext(agentIdx, &constraints, scale);
		}

		Cost admissibleHeuristic(const State& s) {
			return m_env.admissibleHeuristic(s);
		}

		bool isSolution(
				const State& s, Cost g,
				std::unordered_map<State, std::tuple<State, Action, Cost, Cost>, std::hash<State>>& camefrom) {
			return m_env.isSolution(s, g, camefrom);
		}

		void getNeighbors(const State& s, Action act,
											std::vector<Neighbor<State, Action, Cost>>& neighbors, int hardCheck) {
			m_env.getNeighbors(s, act, neighbors, hardCheck);
		}

		State getGoal() { return m_env.getGoal(); }

		int calcIndex(const State& s) { return m_env.calcIndex(s); }

		void onExpandNode(const State& s, Cost fScore, Cost gScore) {
			// std::cout << "LL expand: " << s << std::endl;
			m_env.onExpandLowLevelNode(s, fScore, gScore);
		}

		void onDiscover(const State& /*s*/, Cost /*fScore*/, Cost /*gScore*/) {
			// std::cout << "LL discover: " << s << std::endl;
			// m_env.onDiscoverLowLevel(s, m_agentIdx, m_constraints);
		}

	 private:
		Environment& m_env;
		double scale;
	};

	struct LowLevelDuelEnvironment {
		LowLevelDuelEnvironment(Environment& env, Conflict& conflict, const std::vector<State>& initialStates, double scale)
				: m_env(env), m_conflict(conflict), m_initialStates(initialStates) {}

		bool solve(std::vector<PlanResult<State, Action, double>> &duelSolution) {

			Conflict conflict;

			double scale1 = 1;

			double scale2 = 3;

			while ((scale1 >= 1 || scale2 >= 1)) {

				if (m_env.getDuelConflict(duelSolution, m_conflict.agent1, m_conflict.agent2, conflict)) {  // 若两车存在冲突，或者 scale 未恢复

					// std::cout << "here1" << std::endl;

					if (scale1 >= scale2) {
						// 分配给车辆 1 的冲突
						Constraints c1;
						c1.constraints.emplace(
								Constraint(conflict.time, conflict.s2, conflict.agent2));
						
						// 构造车辆 1 的 lowlevel 
						LowLevelEnvironment llenv1(m_env, conflict.agent1, c1, scale1);
						LowLevelSearch_t lowLevel1(llenv1);

						bool success1 = lowLevel1.search(m_initialStates[conflict.agent1], duelSolution[conflict.agent1], 1);

						scale1--;

						m_env.setLowLevelScale(std::max(scale1, (double)1));
					} else {
						// 分配给车辆 2 的冲突
						Constraints c2;
						c2.constraints.emplace(
								Constraint(conflict.time, conflict.s1, conflict.agent1));
						
						// 构造车辆 2 的 lowlevel 
						LowLevelEnvironment llenv2(m_env, conflict.agent2, c2, scale2);
						LowLevelSearch_t lowLevel2(llenv2);

						scale2--;

						bool success2 = lowLevel2.search(m_initialStates[conflict.agent2], duelSolution[conflict.agent2], 1);
					
						m_env.setLowLevelScale(std::max(scale2, (double)1));
					}
				} else {
					if (scale1 >= scale2) {
						scale1--;
						m_env.setLowLevelScale(std::max(scale1, (double)1));
					} else {
						scale2--;
						m_env.setLowLevelScale(std::max(scale2, (double)1));
					}
				}
			}

			return true;
		}

		Cost admissibleHeuristic(const State& s) {
			return m_env.admissibleHeuristic(s);
		}

		bool isSolution(
				const State& s, Cost g,
				std::unordered_map<State, std::tuple<State, Action, Cost, Cost>, std::hash<State>>& camefrom) {
			return m_env.isSolution(s, g, camefrom);
		}

		void getNeighbors(const State& s, Action act,
											std::vector<Neighbor<State, Action, Cost>>& neighbors, int hardCheck) {
			m_env.getNeighbors(s, act, neighbors, hardCheck);
		}

		State getGoal() { return m_env.getGoal(); }

		int calcIndex(const State& s) { return m_env.calcIndex(s); }

		void onExpandNode(const State& s, Cost fScore, Cost gScore) {
			// std::cout << "LL expand: " << s << std::endl;
			m_env.onExpandLowLevelNode(s, fScore, gScore);
		}

		void onDiscover(const State& /*s*/, Cost /*fScore*/, Cost /*gScore*/) {
			// std::cout << "LL discover: " << s << std::endl;
			// m_env.onDiscoverLowLevel(s, m_agentIdx, m_constraints);
		}

	 private:
		Environment& m_env;
		Conflict&  m_conflict;
		const std::vector<State>& m_initialStates;
	};

 private:
	Environment& m_env;
	typedef HybridAStar<State, Action, Cost, LowLevelEnvironment>
			LowLevelSearch_t;
};

}  // namespace libMultiRobotPlanning
