#include <Eigen/Dense>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <map>
#include <vector>

struct CandidateScore {
  double goal_score;
  double prev_score;
  double expansion_score;
  int data_idx;
};

std::vector<int> compute_pareto_front(std::vector<CandidateScore> candidates) {
  std::sort(candidates.begin(), candidates.end(),
            [](const CandidateScore &a, const CandidateScore &b) {
              if (a.goal_score != b.goal_score) {
                return a.goal_score > b.goal_score;
              }
              if (a.prev_score != b.prev_score) {
                return a.prev_score > b.prev_score;
              }
              return a.expansion_score > b.expansion_score;
            });

  std::map<double, CandidateScore> frontier_2d;
  for (const auto &candidate : candidates) {
    auto it = frontier_2d.lower_bound(candidate.prev_score);
    if (it != frontier_2d.end() &&
        it->second.expansion_score >= candidate.expansion_score) {
      continue;
    }

    while (it != frontier_2d.begin()) {
      auto prev = std::prev(it);
      if (prev->second.expansion_score <= candidate.expansion_score) {
        frontier_2d.erase(prev);
      } else {
        break;
      }
    }

    if (it != frontier_2d.end() &&
        it->first == candidate.prev_score &&
        it->second.expansion_score <= candidate.expansion_score) {
      frontier_2d.erase(it);
    }

    frontier_2d.emplace(candidate.prev_score, candidate);
  }

  std::vector<int> pareto_indices;
  pareto_indices.reserve(frontier_2d.size());
  for (const auto &entry : frontier_2d) {
    pareto_indices.push_back(entry.second.data_idx);
  }
  return pareto_indices;
}

int compute_expansion_score(const Eigen::MatrixXd &unsafe_points,
                            const Eigen::Vector2d &frontier_point, double l_t,
                            double f_max, double lipschitz_L) {
  if (unsafe_points.rows() == 0 || lipschitz_L <= 0.0) {
    return 0;
  }

  const double radius = (f_max - l_t) / lipschitz_L;
  if (radius <= 0.0) {
    return 0;
  }

  const double radius_sq = radius * radius;
  const Eigen::VectorXd dist_sq =
      (unsafe_points.rowwise() - frontier_point.transpose())
          .rowwise()
          .squaredNorm();
  return (dist_sq.array() <= radius_sq).cast<int>().sum();
}

void test_expansion_score_early_reject() {
  Eigen::MatrixXd unsafe_points(2, 2);
  unsafe_points << 1.0, 0.0, 0.0, 1.0;
  Eigen::Vector2d frontier_point(0.0, 0.0);
  int score = compute_expansion_score(unsafe_points, frontier_point,
                                      /*l_t=*/2.0, /*f_max=*/1.0,
                                      /*lipschitz_L=*/1.0);
  assert(score == 0);
}

void test_expansion_score_squared_distance() {
  Eigen::MatrixXd unsafe_points(2, 2);
  unsafe_points << 3.0, 0.0, 0.0, 5.0;
  Eigen::Vector2d frontier_point(0.0, 0.0);
  int score = compute_expansion_score(unsafe_points, frontier_point,
                                      /*l_t=*/1.0, /*f_max=*/5.0,
                                      /*lipschitz_L=*/1.0);
  assert(score == 1);
}

void test_pareto_front_single() {
  std::vector<CandidateScore> candidates = {
      {0.95, 0.05, 0.5, 0}, {0.9, 0.1, 1.0, 1},
      {0.8, 0.2, 2.0, 2},  {0.7, 0.15, 1.5, 3}};
  auto pareto = compute_pareto_front(candidates);
  assert(pareto.size() == 1);
  assert(pareto.front() == 2);
}

void test_pareto_front_tradeoff() {
  std::vector<CandidateScore> candidates = {
      {0.9, 0.3, 1.0, 0}, {0.85, 0.1, 3.0, 1},
      {0.8, 0.2, 0.9, 2}, {0.7, 0.25, 0.8, 3}};
  auto pareto = compute_pareto_front(candidates);
  std::sort(pareto.begin(), pareto.end());
  assert(pareto.size() == 2);
  assert(pareto[0] == 0);
  assert(pareto[1] == 1);
}

int main() {
  test_expansion_score_early_reject();
  test_expansion_score_squared_distance();
  test_pareto_front_single();
  test_pareto_front_tradeoff();
  std::cout << "subgoal_preference_test: all tests passed\n";
  return 0;
}
