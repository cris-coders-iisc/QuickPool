#define BOOST_TEST_MODULE Intersection_Matching

#include <boost/test/data/monomorphic.hpp>
#include <boost/test/data/test_case.hpp>
#include <boost/test/included/unit_test.hpp>

#ifdef Inter_v1
    #include "Intersection_eval.h"
#else
    #ifdef Inter_v2
        #include "Intersection_eval_2.h"
    #else
        #include "Intersection_eval_3.h"
    #endif
#endif

using namespace quickpool;
namespace bdata = boost::unit_test::data;

constexpr int TEST_DATA_MAX_VAL = 1000;
constexpr int SECURITY_PARAM = 128;

struct GlobalFixture {
  GlobalFixture() {
    NTL::ZZ_p::init(NTL::conv<NTL::ZZ>("17816577890427308801"));
  }
};

BOOST_GLOBAL_FIXTURE(GlobalFixture);

BOOST_AUTO_TEST_SUITE(Intersection_Matching)

// testing the functionality of intersection based matching among multiple riders and drivers
BOOST_AUTO_TEST_CASE(intersection_test) {
  NTL::ZZ_pContext ZZ_p_ctx;
  ZZ_p_ctx.save();
  int rider_count = 2;
  int driver_count = 3;
  int nP = rider_count + driver_count;

  #ifdef Inter_v1
    std::vector<std::vector<uint64_t>> all_routes(nP);
  #else
    #ifdef Inter_v2
      __m128i **all_routes = new __m128i*[nP];
    #else
      emp::block **all_routes = new emp::block*[nP];
    #endif
  #endif  

  std::vector<std::future<std::vector<bool>>> parties;
  parties.reserve(nP+1);
  for (int i = 0; i <= nP; ++i) {      
    parties.push_back(std::async(std::launch::async, [&, i]() {          
      ZZ_p_ctx.restore();       
      // auto network = std::make_shared<io::NetIOMP>(i, nP+1, 10000, nullptr, true);  
      auto network = std::make_shared<io::NetIOMP>(i, rider_count, driver_count, 10000, nullptr, true); 
      Intersection_eval iter_eval(i, rider_count, driver_count, network, SECURITY_PARAM, nP);      
      if (i!=0) {
        iter_eval.setInputs();
        #ifdef Inter_v1
          all_routes[i-1] = iter_eval.getInputs();
        #else 
          #ifdef Inter_v2
            all_routes[i-1] = new __m128i[VERTEX_NUM];
            memcpy(all_routes[i-1], iter_eval.getInputs(), VERTEX_NUM*sizeof(emp::block));
          #else
            all_routes[i-1] = new emp::block[VERTEX_NUM];
            memcpy(all_routes[i-1], iter_eval.getInputs(), VERTEX_NUM*sizeof(emp::block)); 
          #endif
        #endif
      }        
      auto res = iter_eval.processIntersectionMatching();
      return res;
    }));
  }

  auto output = parties[0].get();

  std::vector<bool> exp_output;

  #ifdef Inter_v1

    for (size_t r=0; r<rider_count; r++) {
      for (size_t d=0; d<driver_count; d++) {
        std::vector<uint64_t> r_vec = all_routes[r];
        std::vector<uint64_t> d_vec = all_routes[rider_count+d];
        // count number of common values
        std::sort(r_vec.begin(), r_vec.end());
        std::sort(d_vec.begin(), d_vec.end());
        std::vector<int> v_intersection;
        std::set_intersection(
            r_vec.begin(), r_vec.end(),
            d_vec.begin(), d_vec.end(),
            std::back_inserter(v_intersection)
        );
        // check if the count is higher than threshold or not
        if (v_intersection.size()>=DRIVER_RIDER_MATCH_THRESHOLD) {
            exp_output.push_back(1);
        }
        else {
          exp_output.push_back(0);
        }
      }
    }

  #else
    #ifdef Inter_v2

      for (size_t r=0; r<rider_count; r++) {
        for (size_t d=0; d<driver_count; d++) {
          __m128i *r_vec = all_routes[r];
          __m128i *d_vec = all_routes[rider_count+d];
          // count number of common values
          int match_count=0;
          for (size_t k=0; k<VERTEX_NUM; k++) {
              emp::block r_elem = r_vec[k];
              for (size_t l=0; l<VERTEX_NUM; l++) {
                  emp::block d_elem = d_vec[l];
                  if (emp::cmpBlock(&r_elem, &d_elem, 1))
                    match_count++;
              }
          }
          // check if the count is higher than threshold or not
          if (match_count>=DRIVER_RIDER_MATCH_THRESHOLD) {
              exp_output.push_back(1);
          }
          else {
            exp_output.push_back(0);
          }
        }
      }
      for (size_t i=0; i<nP; i++) {
        delete[] all_routes[i];
      }
      delete[] all_routes;

    #else

      for (size_t r=0; r<rider_count; r++) {
        for (size_t d=0; d<driver_count; d++) {
          emp::block *r_vec = all_routes[r];
          emp::block *d_vec = all_routes[rider_count+d];
          // count number of common values
          int match_count=0;
          for (size_t k=0; k<VERTEX_NUM; k++) {
              emp::block r_elem = r_vec[k];
              for (size_t l=0; l<VERTEX_NUM; l++) {
                  emp::block d_elem = d_vec[l];
                  if (emp::cmpBlock(&r_elem, &d_elem, 1))
                    match_count++;
              }
          }
          // check if the count is higher than threshold or not
          if (match_count>=DRIVER_RIDER_MATCH_THRESHOLD) {
              exp_output.push_back(1);
          }
          else {
            exp_output.push_back(0);
          }
        }
      }
      for (size_t i=0; i<nP; i++) {
        delete[] all_routes[i];
      }
      delete[] all_routes;
      
    #endif
  #endif  

  // std::cout << output.size() << " " << exp_output.size() << std::endl;
  // for (size_t k=0; k<exp_output.size(); k++) {
  //   std::cout << "k = " << k << ", output: " << output[k] << ", exp_output: " << exp_output[k] << std::endl;
  // }

  BOOST_TEST(output == exp_output);    
}

BOOST_AUTO_TEST_SUITE_END()


