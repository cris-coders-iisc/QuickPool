#include "netmp.h"
#include "rand_gen_pool.h"
#include "helpers.h"
#include "ED_eval.h"

#define DRIVER_RIDER_MATCH_THRESHOLD 10
#define VERTEX_NUM 4096

namespace quickpool {

class Intersection_eval {
    int id_;
    int rider_count_;
    int driver_count_;
    std::shared_ptr<io::NetIOMP> network_;
    int security_param_;
    RandGenPool rgen_;
    std::shared_ptr<ThreadPool> tpool_;
    emp::block routes[VERTEX_NUM];

public:
    Intersection_eval(int id, int rider_count, int driver_count, std::shared_ptr<io::NetIOMP> network, int security_param, int threads, int seed=200);

    void setInputs();

    emp::block* getInputs();
    
    std::vector<bool> processIntersectionMatching();

};

};