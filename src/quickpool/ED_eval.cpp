#include "ED_eval.h"

namespace quickpool {

ED_eval::ED_eval(int id, int rider_count, int driver_count, std::shared_ptr<io::NetIOMP> network, LevelOrderedCircuit circ, int security_param, int threads, int seed)
    : id_(id),
    rider_count(rider_count),
    driver_count(driver_count),
    network_(network),
    circ_(circ),
    security_param_(security_param),
    seed_(seed)
    {tpool_ = std::make_shared<ThreadPool>(threads);}

// checking if the current party is a rider or not
bool ED_eval::amIRider() {
  return (id_!=0 && id_<=rider_count);
}

// checking if the current party is a driver or not
bool ED_eval::amIDriver() {
  return (id_!=0 && id_>rider_count);
}

// computing the Euclidean distances between start and end positions of a single rider and a single driver
std::vector<Field> ED_eval::pair_matching(const std::unordered_map<wire_t, int>& input_pid_map, const std::unordered_map<wire_t, Field>& inputs, int rider_index, int driver_index) {
    std::vector<Field> res(circ_.outputs.size());  
    if (id_==0 || id_==rider_index || id_==driver_index) {
        OfflineEvaluator eval(id_, rider_count, driver_count, network_, circ_, security_param_, tpool_, seed_);
        auto preproc = eval.run(input_pid_map);
        OnlineEvaluator online_eval(id_, rider_count, driver_count, std::move(network_), std::move(preproc), circ_, security_param_, tpool_, seed_);        
        auto res = online_eval.evaluateCircuit(inputs);
        return res;
    }
    return res;
}

// computing the Euclidean distances between start and end positions of multiple riders and drivers
std::vector<Field> ED_eval::pair_matching(const std::unordered_map<wire_t, int>& input_pid_map, const std::unordered_map<wire_t, Field>& inputs) {
    OfflineEvaluator eval(id_, rider_count, driver_count, network_, circ_, security_param_, tpool_, seed_);
    auto preproc = eval.run(input_pid_map);
    OnlineEvaluator online_eval(id_, rider_count, driver_count, std::move(network_), std::move(preproc), circ_, security_param_, tpool_, seed_);
    auto res = online_eval.evaluateCircuit(inputs);
    return res;
}

// matching between a single driver and rider
std::vector<Field> ED_eval::pair_EDMatching(const std::unordered_map<wire_t, int>& input_pid_map, const std::unordered_map<wire_t, Field>& inputs, int rider_index, int driver_index) { 
    std::vector<Field> output;
    
    if (id_==0 || id_==rider_index || id_==driver_index) {
        // preprocessing phase for computing the Euclidean distances
        OfflineEvaluator eval(id_, rider_count, driver_count, network_, circ_, security_param_, tpool_, seed_);
        auto preproc = eval.run(input_pid_map);

        Field mask0, mask1;
        if (id_==0) {            
            mask0 = preproc.gates[circ_.outputs[0]]->tpmask.secret();
            mask1 = preproc.gates[circ_.outputs[1]]->tpmask.secret();
        }        
        
        //online phase for computing the Euclidean distances
        OnlineEvaluator online_eval(id_, rider_count, driver_count, network_, std::move(preproc), circ_, security_param_, tpool_, seed_);
        online_eval.setInputs(inputs);
        for (size_t i = 0; i < circ_.gates_by_level.size(); ++i) {
            online_eval.evaluateGatesAtDepth(i);
        }

        // DCF to compare if the distances are within the given thresholds
        if (id_==0) {
            uint8_t k_rider0[KEY_LEN], k_rider1[KEY_LEN], k_driver0[KEY_LEN], k_driver1[KEY_LEN];
            std::vector<uint8_t> k_rider(2 * KEY_LEN);
            std::vector<uint8_t> k_driver(2 * KEY_LEN);
            // SP generates the keys for DCF and sends to the rider and driver
            DCF_gen(mask0, k_rider0, k_driver0);
            DCF_gen(mask1, k_rider1, k_driver1);        
            for(int i = 0; i < 2 * KEY_LEN; i++) {
                if(i < KEY_LEN) {
                    k_rider[i] = k_rider0[i];
                    k_driver[i] = k_driver0[i];
                }
                else {
                    k_rider[i] = k_rider1[i-KEY_LEN];
                    k_driver[i] = k_driver1[i - KEY_LEN];
                }
            }            
            network_->send(rider_index, k_rider.data(), k_rider.size() * sizeof(uint8_t));
            network_->send(driver_index, k_driver.data(), k_driver.size() * sizeof(uint8_t));
        }

        if (id_!=0){
            Field masked_val0, masked_val1;
            masked_val0 = online_eval.getwires()[circ_.outputs[0]]; // masked_val should be updated for comparison
            masked_val0 = masked_val0 - (START_MATCH_THRESHOLD * START_MATCH_THRESHOLD);
            masked_val1 = online_eval.getwires()[circ_.outputs[1]];
            masked_val1 = masked_val1 - (END_MATCH_THRESHOLD * END_MATCH_THRESHOLD);

            uint8_t key0[KEY_LEN], key1[KEY_LEN];
            std::vector<uint8_t> key(2 * KEY_LEN, 0);
            network_->recv(0, key.data(), key.size() * sizeof(uint8_t)); // key size may differ.
            for(int i = 0;  i < 2 * KEY_LEN; ++i) {
                if(i < KEY_LEN) {
                    key0[i] = key[i];
                }
                else {
                    key1[i - KEY_LEN] = key[i];
                }
            }

            Field comp_output0, comp_output1;
            if (id_==rider_index) {
                comp_output0 = DCF_eval(0, key0, masked_val0);
                comp_output1 = DCF_eval(0, key1, masked_val1);
            }
            else if (id_==driver_index) {
                comp_output0 = DCF_eval(1, key0, masked_val0);
                comp_output1 = DCF_eval(1, key1, masked_val1);
            }            

            output.push_back(comp_output0);
            output.push_back(comp_output1);
        }
    }
    return output;
}

// matching among multiple drivers and riders
std::vector<Field> ED_eval::pair_EDMatching(const std::unordered_map<wire_t, int>& input_pid_map, const std::unordered_map<wire_t, Field>& inputs) { 
    std::vector<Field> output;
    
    // preprocessing phase for computing the Euclidean distances
    OfflineEvaluator eval(id_, rider_count, driver_count, network_, circ_, security_param_, tpool_, seed_);
    auto preproc = eval.run(input_pid_map);

    std::vector<Field> masks;
    if (id_==0) {
        for (size_t i = 0; i < circ_.outputs.size(); ++i) {
            auto wout = circ_.outputs[i];
            masks.push_back(preproc.gates[wout]->tpmask.secret());
        }     
    }        
    
    // online phase for computing the Euclidean distances
    OnlineEvaluator online_eval(id_, rider_count, driver_count, network_, std::move(preproc), circ_, security_param_, tpool_, seed_);
    online_eval.setInputs(inputs);
    for (size_t i = 0; i < circ_.gates_by_level.size(); ++i) {
        online_eval.evaluateGatesAtDepth(i);
    }

    // DCF to compare if the distances are within the given thresholds
    // std::vector<Field> lengths(rider_count+driver_count,0);
    std::vector<Field> lengths(rider_count+driver_count,driver_count*2);
    if (id_==0) {
        uint8_t k_rider[KEY_LEN], k_driver[KEY_LEN];
        // std::vector<std::vector<uint8_t>> keys_for_parties(rider_count+driver_count);
        std::vector<std::vector<uint8_t>> keys_for_parties(rider_count+driver_count, std::vector<uint8_t>(driver_count*2*KEY_LEN, 0));
        std::vector<std::future<void>> circ_t;
        for (size_t i = 0; i < circ_.outputs.size(); i++) {
            circ_t.push_back(tpool_->enqueue([&,i]() {
                auto wout = circ_.outputs[i];
                int rider_id = circ_.output_owners[wout][0];
                int driver_id = circ_.output_owners[wout][1];
                // std::cout << "output gate count : " << i << ", rider_id = " << rider_id << ", driver_id = " << driver_id << std::endl;
                Field mask = masks[i];
                // SP generates the keys for DCF and sends to riders and drivers
                DCF_gen(mask, k_rider, k_driver);            
                for (size_t j=0; j<KEY_LEN; j++) {
                    // keys_for_parties[rider_id-1].push_back(k_rider[j]);
                    // keys_for_parties[driver_id-1].push_back(k_driver[j]);
                    memcpy(&keys_for_parties[rider_id-1][0]+((driver_id-rider_count-1)*KEY_LEN), k_rider, sizeof(k_rider));
                    memcpy(&keys_for_parties[driver_id-1][0]+((rider_id-1)*KEY_LEN), k_driver, sizeof(k_driver));
                }
                // lengths[rider_id-1]++;
                // lengths[driver_id-1]++;
            }));
        }
        for (auto& f: circ_t) {
            f.get();
        }
        std::vector<std::future<void>> send_t;
        for (size_t i = 1; i <= rider_count+driver_count; i++) {
            send_t.push_back(tpool_->enqueue([&,i]() {
                network_->send(i, keys_for_parties[i-1].data(), keys_for_parties[i-1].size() * sizeof(uint8_t));
            })); 
        }
        for (auto& f: send_t) {
            f.get();
        }
    }

    if (id_!=0){
        std::vector<Field> masked_vals;
        for (size_t i = 0; i < circ_.outputs.size();) {
            auto wires = online_eval.getwires();
            auto wout = circ_.outputs[i++];            
            int rider_id = circ_.output_owners[wout][0];
            int driver_id = circ_.output_owners[wout][1];
            if (id_==rider_id || id_==driver_id) {
                masked_vals.push_back(wires[wout]-(START_MATCH_THRESHOLD * START_MATCH_THRESHOLD));
                auto wout = circ_.outputs[i++];
                masked_vals.push_back(wires[wout]-(END_MATCH_THRESHOLD * END_MATCH_THRESHOLD));
            }            
        }
        std::vector<uint8_t> keys(masked_vals.size() * KEY_LEN);
        network_->recv(0, keys.data(), masked_vals.size() * KEY_LEN * sizeof(uint8_t));
        std::vector<Field> output_share;
        if (amIRider()) {
            for (size_t i = 0; i < masked_vals.size(); i++) {
                Field mask_val = masked_vals[i];
                uint8_t k[KEY_LEN];
                std::copy(keys.begin()+(i*KEY_LEN), keys.begin()+(i*KEY_LEN)+KEY_LEN, k);
                Field out_share = DCF_eval(0, k, mask_val);
                output_share.push_back(out_share);
                mask_val = masked_vals[++i];
                std::copy(keys.begin()+(i*KEY_LEN), keys.begin()+(i*KEY_LEN)+KEY_LEN, k);
                out_share = DCF_eval(0, k, mask_val);
                output_share.push_back(out_share);
            }
        }
        else if (amIDriver()) {
            for (size_t i = 0; i < masked_vals.size(); i++) {
                Field mask_val = masked_vals[i];
                uint8_t k[KEY_LEN];
                std::copy(keys.begin()+(i*KEY_LEN), keys.begin()+(i*KEY_LEN)+KEY_LEN, k);
                Field out_share = DCF_eval(1, k, mask_val);
                output_share.push_back(out_share);
                mask_val = masked_vals[++i];
                std::copy(keys.begin()+(i*KEY_LEN), keys.begin()+(i*KEY_LEN)+KEY_LEN, k);
                out_share = DCF_eval(1, k, mask_val);
                output_share.push_back(out_share);
            }
        }
        // riders and drivers send the shares of DCF output to SP for reconstruction
        network_->send(0, output_share.data(), output_share.size() * sizeof(Field));
    }

    if (id_==0) {
        std::vector<std::vector<bool>> match(rider_count, std::vector<bool>(driver_count));
        std::vector<std::vector<Field>> output_shares(rider_count+driver_count);
        // network_->flush();
        network_->flush(rider_count, driver_count);
        std::vector<std::future<void>> recv_t;
        for (size_t i = 1; i <= rider_count+driver_count; i++) {
            recv_t.push_back(tpool_->enqueue([&,i]() {
                output_shares[i-1].resize(lengths[i-1]);
                network_->recv(i, output_shares[i-1].data(), lengths[i-1]*sizeof(Field));
            }));
        }
        for (auto& f: recv_t) {
            f.get();
        }
        std::vector<size_t> index(rider_count+driver_count, 0);
        for (size_t i = 0; i < circ_.outputs.size(); i++) {
            auto wout = circ_.outputs[i];
            int rider_id = circ_.output_owners[wout][0];
            int driver_id = circ_.output_owners[wout][1];
            // SP recontructs the DCF output
            Field start_comp_output = output_shares[rider_id-1][index[rider_id-1]] + output_shares[driver_id-1][index[driver_id-1]];
            index[rider_id-1]++;
            index[driver_id-1]++;
            i++;
            // SP recontructs the DCF output
            Field end_comp_output = output_shares[rider_id-1][index[rider_id-1]] + output_shares[driver_id-1][index[driver_id-1]];
            index[rider_id-1]++;
            index[driver_id-1]++;
            output.push_back(start_comp_output*end_comp_output);
            match[rider_id-1][driver_id-rider_count-1] = bool(start_comp_output*end_comp_output);
        }
        // SP locally runs the algorithm for finding the maximal matching
        int maxBPM_res = maxBPM(match); // just for benchmarking
    }

    return output;

}


bool bpm(std::vector<std::vector<bool>> bpGraph, int u,
        std::vector<bool> seen, std::vector<int> matchR)
{
    int M = bpGraph.size();
    int N = bpGraph[0].size();
    for (int v = 0; v < N; v++) {
        if (bpGraph[u][v] && !seen[v]) {
            seen[v] = true; 
            if (matchR[v] < 0 || bpm(bpGraph, matchR[v],
                                    seen, matchR)) {
                matchR[v] = u;
                return true;
            }
        }
    }
    return false;
}

// Gale-Shapley algorithm for finding maximal matching in a bipartite graph
int maxBPM(std::vector<std::vector<bool>> bpGraph) {
    int M = bpGraph.size();
    int N = bpGraph[0].size();

    std::vector<int> matchR(N, -1);

    int result = 0; 
    for (int u = 0; u < M; u++) {
        std::vector<bool> seen(N,0);

        if (bpm(bpGraph, u, seen, matchR))
            result++;
    }
    return result;
}
    
}; // namespace quickpool

