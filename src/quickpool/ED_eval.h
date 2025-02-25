#pragma once

#include "ED_offline_eval.h"
#include "ED_online_eval.h"
#include "fss.h"

using namespace common::utils;

#define START_MATCH_THRESHOLD (Field)50
#define END_MATCH_THRESHOLD (Field)50

namespace quickpool {

class ED_eval {
    int id_;
    int rider_count;
    int driver_count;
    std::shared_ptr<io::NetIOMP> network_;
    LevelOrderedCircuit circ_;
    int security_param_;
    std::shared_ptr<ThreadPool> tpool_;
    int seed_;

public:
    ED_eval(int id, int rider_count, int driver_count, std::shared_ptr<io::NetIOMP> network, LevelOrderedCircuit circ_, int security_param, int threads, int seed=200);

    bool amIRider();

    bool amIDriver();

    std::vector<Field> pair_matching(const std::unordered_map<wire_t, int>& input_pid_map, const std::unordered_map<wire_t, Field>& inputs, int rider_index, int driver_index);

    std::vector<Field> pair_matching(const std::unordered_map<wire_t, int>& input_pid_map, const std::unordered_map<wire_t, Field>& inputs);

    std::vector<Field> pair_EDMatching(const std::unordered_map<wire_t, int>& input_pid_map, const std::unordered_map<wire_t, Field>& inputs, int rider_index, int driver_index);

    std::vector<Field> pair_EDMatching(const std::unordered_map<wire_t, int>& input_pid_map, const std::unordered_map<wire_t, Field>& inputs);

};


bool bpm(std::vector<std::vector<bool>> bpGraph, int u, std::vector<bool> seen, std::vector<int> matchR);

int maxBPM(std::vector<std::vector<bool>> bpGraph);

}; // namespace quickpool