#include "Intersection_eval.h"

namespace quickpool {

Intersection_eval::Intersection_eval(int id, int rider_count, int driver_count, std::shared_ptr<io::NetIOMP> network, int security_param, int threads, int seed) : 
    id_(id), 
    rider_count_(rider_count),
    driver_count_(driver_count),
    network_(network),
    security_param_(security_param),
    rgen_(id, seed)
    {tpool_ = std::make_shared<ThreadPool>(threads);}

// setting random inputs for the routes of length VERTEX_NUM
void Intersection_eval::setInputs() {
    if (id_!=0) {
        // sample the route vertices
        // int max_val = INT32_MAX;
        int max_val = 5000;
        int count = VERTEX_NUM; // number of route vertices

        std::vector<uint64_t> sample_space(max_val);
        std::iota(sample_space.begin(), sample_space.end(), 1);
        std::random_shuffle(sample_space.begin(),sample_space.end());

        uint64_t val1 = uint64_t(sample_space[0]);
        for (size_t i=0, j=1; i<count; i++, j++) {            
            uint64_t val2 = uint64_t(sample_space[j]);
            emp::block val = emp::makeBlock(val1,0) ^ emp::makeBlock(0,val2); // making a pair of the points
            routes[i] = val;
            val1 = val2;
        }

        // std::cout << "ID: " << id_ << std::endl;
        // for (auto route : routes) {
        //     print128_num(route);
        // }
        // std::cout << "\n==========================" << std::endl;
    }    
}

// returns the routes
emp::block* Intersection_eval::getInputs() {
    return routes;
}

std::vector<bool> Intersection_eval::processIntersectionMatching(){
    std::vector<bool> output;
    if (id_!=0) { // I am not SP
        // random key sampling
        emp::block key_own;
        rgen_.self().random_data(&key_own, sizeof(emp::block));
        emp::block *keys;
        // exchange keys and prepare the keys to be used in PRF evaluation
        if (id_<=rider_count_) { // I am a rider
            keys = new emp::block[driver_count_];
            for (size_t i=rider_count_+1; i<=rider_count_+driver_count_; i++) {
                // send the sampled key to each driver
                network_->send(i, &key_own, sizeof(emp::block));
                // receive key from each driver
                emp::block key_other;
                network_->recv(i, &key_other, sizeof(emp::block));
                keys[i-rider_count_-1] = _mm_xor_si128(key_own,key_other);
            }
        }
        else { // I am a driver
            keys = new emp::block[rider_count_];
            for (size_t i=1; i<=rider_count_; i++) {
                // send the sampled key to each rider
                network_->send(i, &key_own, sizeof(emp::block));
                // receive key from each rider
                emp::block key_other;
                network_->recv(i, &key_other, sizeof(emp::block));
                keys[i-1] = _mm_xor_si128(key_own,key_other);
            }
        }
        
        // PRF evaluation of route vertices
        if (id_<=rider_count_) { // I am a rider
            // run PRF on routes vertices for each driver
            emp::block prf_result[driver_count_*VERTEX_NUM];
            for (size_t i=0; i<driver_count_; i++) {
                emp::PRP aes(keys[i]);
                memcpy(prf_result+(i*VERTEX_NUM), routes, sizeof(routes));
                aes.permute_block(prf_result+(i*VERTEX_NUM), VERTEX_NUM);
            }
            // send prf results to SP
            network_->send(0, prf_result, sizeof(prf_result));
        }
        else { // I am a driver
            // run PRF on routes vertices for each rider
            emp::block prf_result[rider_count_*VERTEX_NUM];
            for (size_t i=0, k=0; i<rider_count_; i++) {
                emp::PRP aes(keys[i]);
                memcpy(prf_result+(i*VERTEX_NUM), routes, sizeof(routes));
                aes.permute_block(prf_result+(i*VERTEX_NUM), VERTEX_NUM);
            }
            // send prf results to SP
            network_->send(0, prf_result, sizeof(prf_result));
        }
        delete[] keys;
    }
    else { // I am SP
        output.resize(rider_count_*driver_count_);
        std::vector<std::vector<bool>> match(rider_count_, std::vector<bool>(driver_count_));
        emp::block *prf_results_riders = new emp::block[rider_count_*driver_count_*VERTEX_NUM];
        emp::block *prf_results_drivers = new emp::block[rider_count_*driver_count_*VERTEX_NUM];
        std::vector<std::future<void>> recv_rider_t, recv_driver_t;
        for (size_t i=1; i<=rider_count_; i++) { // receive from riders
            recv_rider_t.push_back(tpool_->enqueue([&,i]() {
                network_->recv(i, prf_results_riders+(i-1)*driver_count_*VERTEX_NUM, driver_count_*VERTEX_NUM*sizeof(emp::block));
            }));
        }
        for (size_t i=1; i<=driver_count_; i++) { // receive from drivers
            recv_driver_t.push_back(tpool_->enqueue([&,i]() {
                network_->recv(i+rider_count_, prf_results_drivers+(i-1)*rider_count_*VERTEX_NUM, rider_count_*VERTEX_NUM*sizeof(emp::block));
            }));
        }        
        for (auto& f: recv_rider_t) {
            f.get();
        }
        for (auto& f: recv_driver_t) {
            f.get();
        }
        std::vector<std::future<void>> match_t;
        // for each pair of rider and driver check if the match_count is greater than threshold or not
        for (size_t i=0; i<rider_count_; i++) {
            for (size_t j=0; j<driver_count_; j++) {
                match_t.push_back(tpool_->enqueue([&,i,j]() {
                    // count number of common values
                    int match_count=0;
                    size_t r_index = i*driver_count_*VERTEX_NUM+j*VERTEX_NUM;
                    size_t d_index = j*rider_count_*VERTEX_NUM+i*VERTEX_NUM;
                    for (size_t k=0; k<VERTEX_NUM; k++) {
                        emp::block r_elem = prf_results_riders[r_index+k];
                        for (size_t l=0; l<VERTEX_NUM; l++) {
                            emp::block d_elem = prf_results_drivers[d_index+l];
                            if (emp::cmpBlock(&r_elem, &d_elem, 1))
                                match_count++; 
                        }
                    }
                    // check if the count is higher than threshold or not
                    if (match_count>=DRIVER_RIDER_MATCH_THRESHOLD) {
                        output[i*driver_count_+j] = 1;
                        match[i][j] = true;
                    }
                    else {
                        output[i*driver_count_+j] = 0;
                        match[i][j] = false;
                    }
                }));                
            }
        }
        for (auto& f: match_t) {
            f.get();
        }                
        // SP locally runs the algorithm for finding the maximal matching
        int maxBPM_res = maxBPM(match); // just for benchmarking
        delete[] prf_results_riders;
        delete[] prf_results_drivers;    
    }
    
    return output;
}

};