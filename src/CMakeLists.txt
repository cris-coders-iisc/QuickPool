add_library(Quickpool
            utils/circuit.cpp 
            utils/types.cpp
            utils/helpers.cpp
            io/netmp.cpp
            funshade/aes.cpp
            funshade/fss.cpp
            quickpool/rand_gen_pool.cpp
            quickpool/ED_offline_eval.cpp
            quickpool/ED_online_eval.cpp
            quickpool/ED_eval.cpp
            quickpool/Intersection_eval.cpp
            )
            
target_include_directories(Quickpool PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/io ${CMAKE_CURRENT_SOURCE_DIR}/utils ${CMAKE_CURRENT_SOURCE_DIR}/funshade ${CMAKE_CURRENT_SOURCE_DIR}/quickpool)
target_link_libraries(Quickpool PUBLIC Boost::system EMPTool NTL GMP)