#include "ED_offline_eval.h"

namespace quickpool {

OfflineEvaluator::OfflineEvaluator(int my_id, int rider_count, int driver_count,
                                   std::shared_ptr<io::NetIOMP> network,
                                   LevelOrderedCircuit circ,
                                   int security_param, int threads, int seed)
    : id_(my_id),
      rider_count(rider_count),
      driver_count(driver_count),
      security_param_(security_param),
      rgen_(my_id, seed), 
      network_(std::move(network)),
      circ_(std::move(circ)),
      preproc_(circ.num_gates)
      {tpool_ = std::make_shared<ThreadPool>(threads);}

OfflineEvaluator::OfflineEvaluator(int my_id, int rider_count, int driver_count,
                                   std::shared_ptr<io::NetIOMP> network,
                                   LevelOrderedCircuit circ,
                                   int security_param, std::shared_ptr<ThreadPool> tpool, int seed)
    : id_(my_id),
      rider_count(rider_count),
      driver_count(driver_count),
      security_param_(security_param),
      rgen_(my_id, seed), 
      network_(std::move(network)),
      circ_(std::move(circ)),
      preproc_(circ.num_gates),
      tpool_(std::move(tpool)) {}

// checking if the current party is a rider or not
bool OfflineEvaluator::amIRider() {
  return (id_!=0 && id_<=rider_count);
}

// checking if the current party is a driver or not
bool OfflineEvaluator::amIDriver() {
  return (id_!=0 && id_>rider_count);
}

// checking if the input dealer is a rider or not
bool OfflineEvaluator::isDealerRider(int dealer) {
  return (dealer!=0 && dealer<=rider_count);
}

// checking if the input dealer is a driver or not
bool OfflineEvaluator::isDealerDriver(int dealer) {
  return (dealer!=0 && dealer>rider_count);
}

// SP samples a random value and secret-shares among the rider and the driver
void OfflineEvaluator::randomShare(int rider_id, int driver_id,
                                  RandGenPool& rgen, io::NetIOMP& network,
                                  AddShare<Field>& share, TPShare<Field>& tpShare) {

  Field val = Field(0);
  
  if(id_ == 0) {
    share.pushValue(Field(0));
    tpShare.pushValues(Field(0));
    // randomizeZZp(rgen.pi(rider_id), val, sizeof(Field));
    randomize(rgen.pi(rider_id), val, sizeof(Field));
    tpShare.pushValues(val);
    // randomizeZZp(rgen.pi(driver_id), val, sizeof(Field));
    randomize(rgen.pi(driver_id), val, sizeof(Field));
    tpShare.pushValues(val);    
  }
  else if (id_ == rider_id || id_ == driver_id) {
    // randomizeZZp(rgen.p0(), val, sizeof(Field));
    randomize(rgen.p0(), val, sizeof(Field));
    share.pushValue(val);
  }

}

// SP shares a secret among the rider and the driver
void OfflineEvaluator::randomShareSecret(int rider_id, int driver_id,
                                        RandGenPool& rgen, io::NetIOMP& network,
                                        AddShare<Field>& share, TPShare<Field>& tpShare,
                                        Field secret, std::vector<std::vector<Field>>& rand_sh_sec, 
                                        size_t& idx_rand_sh_sec) {
  Field val = Field(0);
  Field valn = Field(0);
  
  if(id_ == 0) {
    share.pushValue(Field(0));
    tpShare.pushValues(Field(0));
    // randomizeZZp(rgen.pi(rider_id), val, sizeof(Field));
    randomize(rgen.pi(rider_id), val, sizeof(Field));
    tpShare.pushValues(val);
    valn = secret - val;
    tpShare.pushValues(valn);
    rand_sh_sec[driver_id-rider_count-1].push_back(valn);
  }
  else if(id_ == rider_id) {
    // randomizeZZp(rgen.p0(), val, sizeof(Field));
    randomize(rgen.p0(), val, sizeof(Field));
    share.pushValue(val);
  }
  else if(id_ == driver_id) {
    valn = rand_sh_sec[driver_id-rider_count-1][idx_rand_sh_sec];
    idx_rand_sh_sec++;
    share.pushValue(valn);
  }
}

// SP and dealer sample a common random value and SP secret-shares among the rider and the driver
void OfflineEvaluator::randomShareWithParty(int dealer, int rider_id,  
                                          int driver_id, RandGenPool& rgen,
                                          io::NetIOMP& network, AddShare<Field>& share,
                                          TPShare<Field>& tpShare, Field& secret, 
                                          std::vector<std::vector<Field>>& rand_sh_party, 
                                          size_t& idx_rand_sh_party) {
                                             
                                            
  Field val = Field(0);
  Field valn = Field(0);

  if( id_ == 0) {
    if(dealer != 0) {
      // randomizeZZp(rgen.pi(dealer), secret, sizeof(Field));
      randomize(rgen.pi(dealer), secret, sizeof(Field));
    }
    else { // this will never occur
      // randomizeZZp(rgen.self(), secret, sizeof(Field));
      randomize(rgen.self(), secret, sizeof(Field));
    }    
    share.pushValue(Field(0));
    tpShare.pushValues(Field(0));
    // randomizeZZp(rgen.pi(rider_id), val, sizeof(Field));
    randomize(rgen.pi(rider_id), val, sizeof(Field));
    tpShare.pushValues(val);
    valn = secret - val;
    rand_sh_party[driver_id-rider_count-1].push_back(valn);
    tpShare.pushValues(valn);
  }
  else {
    if(id_ == dealer) {
      // randomizeZZp(rgen.p0(), secret, sizeof(Field));
      randomize(rgen.p0(), secret, sizeof(Field));

    }
    if(id_ == rider_id) {
      // randomizeZZp(rgen.p0(), val, sizeof(Field));
      randomize(rgen.p0(), val, sizeof(Field));
      share.pushValue(val);
    }
    else if (id_ == driver_id) {           
      valn = rand_sh_party[driver_id-rider_count-1][idx_rand_sh_party];
      idx_rand_sh_party++;
      share.pushValue(valn);
    }
  }
}

void OfflineEvaluator::setWireMasksParty(
                    const std::unordered_map<wire_t, int>& input_pid_map, 
                    std::vector<std::vector<Field>>& rand_sh_sec,
                    std::vector<std::vector<Field>>& rand_sh_party) {
    
  size_t idx_rand_sh_sec = 0;
  size_t idx_rand_sh_party = 0;
  
  for (const auto& level : circ_.gates_by_level) {
    for (const auto& gate : level) {
      switch (gate->type) {
        case GateType::kInp: {
          auto pregate = std::make_unique<PreprocInput<Field>>();
          auto dealer = input_pid_map.at(gate->out);
          auto rider_id = gate->rider_id;
          auto driver_id = gate->driver_id;
          pregate->pid = dealer;
          randomShareWithParty(dealer, rider_id, driver_id, rgen_, *network_, pregate->mask, pregate->tpmask, pregate->mask_value, rand_sh_party, idx_rand_sh_party);
          preproc_.gates[gate->out] = std::move(pregate);
          break;
        }

        case GateType::kAdd: {          
          const auto* g = static_cast<FIn2Gate*>(gate.get());
          const auto& mask_in1 = preproc_.gates[g->in1]->mask;
          const auto& tpmask_in1 = preproc_.gates[g->in1]->tpmask;
          const auto& mask_in2 = preproc_.gates[g->in2]->mask;
          const auto& tpmask_in2 = preproc_.gates[g->in2]->tpmask;
          preproc_.gates[gate->out] =
              std::make_unique<PreprocGate<Field>>((mask_in1 + mask_in2), (tpmask_in1 + tpmask_in2));          
          break;
        }

        case GateType::kConstAdd: {
          const auto* g = static_cast<ConstOpGate<Field>*>(gate.get());
          const auto& mask = preproc_.gates[g->in]->mask;
          const auto& tpmask = preproc_.gates[g->in]->tpmask;
          preproc_.gates[gate->out] =
              std::make_unique<PreprocGate<Field>>((mask), (tpmask));
          break;
        }

        case GateType::kConstMul: {
          const auto* g = static_cast<ConstOpGate<Field>*>(gate.get());
          const auto& mask = preproc_.gates[g->in]->mask * g->cval;
          const auto& tpmask = preproc_.gates[g->in]->tpmask * g->cval;
          preproc_.gates[gate->out] =
              std::make_unique<PreprocGate<Field>>((mask), (tpmask));
          break;
        }

        case GateType::kSub: {
          const auto* g = static_cast<FIn2Gate*>(gate.get());
          const auto& mask_in1 = preproc_.gates[g->in1]->mask;
          const auto& tpmask_in1 = preproc_.gates[g->in1]->tpmask;
          const auto& mask_in2 = preproc_.gates[g->in2]->mask;
          const auto& tpmask_in2 = preproc_.gates[g->in2]->tpmask;
          preproc_.gates[gate->out] =
              std::make_unique<PreprocGate<Field>>((mask_in1 - mask_in2),(tpmask_in1 - tpmask_in2));          
          break;
        }

        case GateType::kMul: {
          preproc_.gates[gate->out] = std::make_unique<PreprocMultGate<Field>>();
          const auto* g = static_cast<FIn2Gate*>(gate.get());
          const auto& mask_in1 = preproc_.gates[g->in1]->mask;
          const auto& tpmask_in1 = preproc_.gates[g->in1]->tpmask;
          const auto& mask_in2 = preproc_.gates[g->in2]->mask;
          const auto& tpmask_in2 = preproc_.gates[g->in2]->tpmask;
          auto rider_id = g->rider_id;
          auto driver_id = g->driver_id;
          Field tp_prod;
          if(id_ == 0) {tp_prod = tpmask_in1.secret() * tpmask_in2.secret();}
          TPShare<Field> tprand_mask;
          AddShare<Field> rand_mask;
          randomShare(rider_id, driver_id, rgen_, *network_, rand_mask, tprand_mask);

          TPShare<Field> tpmask_product;
          AddShare<Field> mask_product; 
          randomShareSecret(rider_id, driver_id, rgen_, *network_, mask_product, tpmask_product, tp_prod, rand_sh_sec, idx_rand_sh_sec);
          preproc_.gates[gate->out] = std::move(std::make_unique<PreprocMultGate<Field>>
                              (rand_mask, tprand_mask, mask_product, tpmask_product));
          break;
        }

        case GateType::kDotprod: {
          preproc_.gates[gate->out] = std::make_unique<PreprocDotpGate<Field>>();
          const auto* g = static_cast<SIMDGate*>(gate.get());
          Field mask_prod = Field(0);
          if(id_ ==0) {
            for(size_t i = 0; i < g->in1.size(); i++) {
              mask_prod += preproc_.gates[g->in1[i]]->tpmask.secret() * preproc_.gates[g->in2[i]]->tpmask.secret();
            }
          }
          auto rider_id = g->rider_id;
          auto driver_id = g->driver_id;
          TPShare<Field> tprand_mask;
          AddShare<Field> rand_mask;
          randomShare(rider_id, driver_id, rgen_, *network_, rand_mask, tprand_mask);

          TPShare<Field> tpmask_product;
          AddShare<Field> mask_product; 
          randomShareSecret(rider_id, driver_id, rgen_, *network_, mask_product, tpmask_product, mask_prod, rand_sh_sec, idx_rand_sh_sec);
                                
          preproc_.gates[gate->out] = std::move(std::make_unique<PreprocDotpGate<Field>>
                              (rand_mask, tprand_mask, mask_product, tpmask_product));
          break;
        }
	
    default: {
          break;
        }
      }
    }
  }
}


void OfflineEvaluator::setWireMasks(
  const std::unordered_map<wire_t, int>& input_pid_map) {
    
  std::vector<std::vector<Field>> rand_sh_sec(driver_count);
  std::vector<std::vector<Field>> rand_sh_party(driver_count);
      
  if(!amIDriver()) {
    setWireMasksParty(input_pid_map, rand_sh_sec, rand_sh_party);

    if(id_ == 0) {

      std::vector<std::future<void>> offline_t;
      
      for (int driver=0; driver<driver_count; driver++){        
        offline_t.push_back(tpool_->enqueue([&,driver]() {
          size_t rand_sh_sec_num = rand_sh_sec[driver].size();      
          size_t rand_sh_party_num = rand_sh_party[driver].size();
          size_t arith_comm = rand_sh_sec_num + rand_sh_party_num;
        
          std::vector<size_t> lengths(3);
          lengths[0] = arith_comm;
          lengths[1] = rand_sh_sec_num;
          lengths[2] = rand_sh_party_num;

          network_->send(driver+rider_count+1, lengths.data(), sizeof(size_t) * lengths.size());

          std::vector<Field> offline_arith_comm(arith_comm);
        
          for(size_t i = 0; i < rand_sh_sec_num; i++) {
            offline_arith_comm[i] = rand_sh_sec[driver][i];
          }
          for(size_t i = 0; i < rand_sh_party_num; i++) {
            offline_arith_comm[rand_sh_sec_num + i] = rand_sh_party[driver][i];
          }
          network_->send(driver+rider_count+1, offline_arith_comm.data(), sizeof(Field) * arith_comm);
        }));        
      }
      for (auto& f: offline_t) {
        f.get();
      }
    }
  }
  else {
    std::vector<size_t> lengths(3);
    network_->recv(0, lengths.data(), sizeof(size_t) * lengths.size());

    size_t arith_comm = lengths[0];
    size_t rand_sh_sec_num = lengths[1];
    size_t rand_sh_party_num = lengths[2];

    std::vector<Field> offline_arith_comm(arith_comm);
    
    network_->recv(0, offline_arith_comm.data(), sizeof(Field) * arith_comm);

    rand_sh_sec[id_-rider_count-1].resize(rand_sh_sec_num);
    for(int i = 0; i < rand_sh_sec_num; i++) {
      rand_sh_sec[id_-rider_count-1][i] = offline_arith_comm[i];
    }
    
    rand_sh_party[id_-rider_count-1].resize(rand_sh_party_num);
    for(int i = 0; i < rand_sh_party_num; i++) {
      rand_sh_party[id_-rider_count-1][i] = offline_arith_comm[rand_sh_sec_num + i];
    }
    
    setWireMasksParty(input_pid_map, rand_sh_sec, rand_sh_party);
  }  
}

PreprocCircuit<Field> OfflineEvaluator::getPreproc() {
  return std::move(preproc_);
}

// run the offline phase
PreprocCircuit<Field> OfflineEvaluator::run(
    const std::unordered_map<wire_t, int>& input_pid_map) {
  setWireMasks(input_pid_map);
  return std::move(preproc_);  
}

};  // namespace quickpool

