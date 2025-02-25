#include <boost/program_options.hpp>

#include "utils.h"
#include "Intersection_eval.h"

using namespace quickpool;
using json = nlohmann::json;
namespace bpo = boost::program_options;

// benchmarking the secure intersection based matching protocol, proposed in QuickPool
void benchmark(const bpo::variables_map &opts)
{
    bool save_output = false;
    std::string save_file;
    if (opts.count("output") != 0)
    {
        save_output = true;
        save_file = opts["output"].as<std::string>();
    }
    
    auto riderCount = opts["rider-count"].as<size_t>();
    auto driverCount = opts["driver-count"].as<size_t>();
    auto pid = opts["pid"].as<size_t>();
    auto security_param = opts["security-param"].as<size_t>();
    auto threads = opts["threads"].as<size_t>();
    auto seed = opts["seed"].as<size_t>();
    auto repeat = opts["repeat"].as<size_t>();
    auto port = opts["port"].as<int>();

    int nP = riderCount + driverCount;

    // establishing the network connection amongst the parties
    std::shared_ptr<io::NetIOMP> network = nullptr;
    if (opts["localhost"].as<bool>())
    {
        // network = std::make_shared<io::NetIOMP>(pid, nP + 1, port, nullptr, true);
        network = std::make_shared<io::NetIOMP>(pid, riderCount, driverCount, port, nullptr, true);
    }
    else
    {
        std::ifstream fnet(opts["net-config"].as<std::string>());
        if (!fnet.good())
        {
            fnet.close();
            throw std::runtime_error("Could not open network config file");
        }
        json netdata;
        fnet >> netdata;
        fnet.close();

        std::vector<std::string> ipaddress(nP + 1);
        // std::array<char *, 5> ip{};
        char **ip = new char*[nP+1];
        for (size_t i = 0; i < nP + 1; ++i)
        {
            ipaddress[i] = netdata[i].get<std::string>();
            ip[i] = ipaddress[i].data();
        }

        // network = std::make_shared<io::NetIOMP>(pid, nP + 1, port, ip.data(), false);
        // network = std::make_shared<io::NetIOMP>(pid, riderCount, driverCount, port, ip.data(), false);
        network = std::make_shared<io::NetIOMP>(pid, riderCount, driverCount, port, ip, false);
    }

    json output_data;
    output_data["details"] = {{"rider-count", riderCount},
                                {"driver-count", driverCount},
                                {"pid", pid},
                                {"security_param", security_param},
                                {"threads", threads},
                                {"seed", seed},
                                {"repeat", repeat}};
    output_data["benchmarks"] = json::array();

    std::cout << "--- Details ---\n";
    for (const auto &[key, value] : output_data["details"].items())
    {
        std::cout << key << ": " << value << "\n";
    }
    std::cout << std::endl;

    for (size_t r = 0; r < repeat; ++r)
    {

        Intersection_eval quickpool_int(pid, riderCount, driverCount, network, security_param, threads, seed);

        // setting random inputs
        quickpool_int.setInputs();
        
        StatsPoint start(*network);
        
        // calling the function for securely executing intersection based matching
        quickpool_int.processIntersectionMatching();

        StatsPoint end(*network);
        auto rbench = end - start;
        output_data["benchmarks"].push_back(rbench);

        size_t bytes_sent = 0;
        for (const auto &val : rbench["communication"])
        {
            bytes_sent += val.get<int64_t>();
        }

        std::cout << "--- Repetition " << r + 1 << " ---\n";
        std::cout << "time: " << rbench["time"] << " ms\n";
        std::cout << "sent: " << bytes_sent << " bytes\n";

        std::cout << std::endl;
    }
    output_data["stats"] = {{"peak_virtual_memory", peakVirtualMemory()},
                            {"peak_resident_set_size", peakResidentSetSize()}};

    std::cout << "--- Statistics ---\n";
    for (const auto &[key, value] : output_data["stats"].items())
    {
        std::cout << key << ": " << value << "\n";
    }
    std::cout << std::endl;

    if (save_output)
    {
        saveJson(output_data, save_file);
    }
}

bpo::options_description programOptions()
{
    bpo::options_description desc("Following options are supported by config file too.");
    desc.add_options()
        ("rider-count,r", bpo::value<size_t>()->required(), "Number of riders.")
        ("driver-count,d", bpo::value<size_t>()->required(), "Number of drivers.")
        ("pid,p", bpo::value<size_t>()->required(), "Party ID.")
        ("security-param", bpo::value<size_t>()->default_value(128), "Security parameter in bits.")
        ("threads,t", bpo::value<size_t>()->default_value(6), "Number of threads (recommended 6).")
        ("seed", bpo::value<size_t>()->default_value(200), "Value of the random seed.")
        ("net-config", bpo::value<std::string>(), "Path to JSON file containing network details of all parties.")
        ("localhost", bpo::bool_switch(), "All parties are on same machine.")
        ("port", bpo::value<int>()->default_value(10000), "Base port for networking.")
        ("output,o", bpo::value<std::string>(), "File to save benchmarks.")
        ("repeat,re", bpo::value<size_t>()->default_value(1), "Number of times to run benchmarks.");

  return desc;
}

int main(int argc, char *argv[])
{
    ZZ_p::init(conv<ZZ>("17816577890427308801"));
    auto prog_opts(programOptions());

    bpo::options_description cmdline(
      "Benchmark for Endpoint Distance Matching.");
    cmdline.add(prog_opts);
    cmdline.add_options()(
        "config,c", bpo::value<std::string>(),
        "configuration file for easy specification of cmd line arguments")(
        "help,h", "produce help message");

    bpo::variables_map opts;
    bpo::store(bpo::command_line_parser(argc, argv).options(cmdline).run(), opts);

    if (opts.count("help") != 0)
    {
        std::cout << cmdline << std::endl;
        return 0;
    }

    if (opts.count("config") > 0)
    {
        std::string cpath(opts["config"].as<std::string>());
        std::ifstream fin(cpath.c_str());

        if (fin.fail())
        {
            std::cerr << "Could not open configuration file at " << cpath << "\n";
            return 1;
        }

        bpo::store(bpo::parse_config_file(fin, prog_opts), opts);
    }

    try
    {
        bpo::notify(opts);

        if (!opts["localhost"].as<bool>() && (opts.count("net-config") == 0))
        {
            throw std::runtime_error("Expected one of 'localhost' or 'net-config'");
        }
    }
    catch (const std::exception &ex)
    {
        std::cerr << ex.what() << std::endl;
        return 1;
    }

    try
    {
        benchmark(opts);
    }
    catch (const std::exception &ex)
    {
        std::cerr << ex.what() << "\nFatal error" << std::endl;
        return 1;
    }

    return 0;
}
