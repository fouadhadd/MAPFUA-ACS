#include <iostream>
#include <boost/program_options.hpp>
#include "include/common.hpp"
#include "include/scenario.hpp"
#include "include/ACS/acs.hpp"
#include <cstdlib>
#include <iomanip>
#include <filesystem>


auto parse_args(int argc, char** argv){
    namespace po = boost::program_options;
    po::options_description desc("Allowed options");
    constexpr static const char* allowed_solvers[] = {"ACS"};
    desc.add_options()
    ("help", "produce help message")
    ("map,m", po::value<std::string>()->required(), "map file ([map_name].map)")
    ("scenario,s", po::value<std::string>()->required(), "scenario file (.scen)")
    ("alg,a", po::value<std::string>()->required(), "solver's algorithm to use (NATCBS)")
    ("time,t", po::value<int>()->default_value(60), "time limit in seconds (default 60)")
    ("output,o", po::value<std::string>()->default_value("results.csv"), "results output file (default results.csv)")
    ("v", po::value<int>()->default_value(1), "verbosity level: 0 - minimal, 1 - normal (default), 2 - debug");

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);

        if (vm.count("help")) {
            std::cout << desc << std::endl;
            exit(0);
        }

        if (std::count(std::begin(allowed_solvers), std::end(allowed_solvers), vm["alg"].as<string>()) == 0){
            throw po::error("invalid solver");
        }

        if (vm["v"].as<int>() < 0 || vm["v"].as<int>() > 2){
            throw po::error("invalid verbosity level");
        }

    } catch (po::error &e) {
        std::cerr << e.what() << std::endl << std::endl;
        std::cerr << desc << std::endl;
        exit(-1);
    }
    return vm;
}

shared_ptr<Scenario> read_scenario(const string& map_file, const string& scenario_file){
    std::ifstream map_stream(map_file);

    if (!map_stream) {
        std::cerr << "Unable to open map file" << std::endl;
        exit(1);
    }

    int map_rows, map_cols;
    map_stream >> map_rows >> map_cols;
    vector<vector<CellType>> map(map_rows, vector<CellType>(map_cols, PASSABLE));

    string row;
    for (int i = 0; i < map_rows; ++i) {
        try {
            map_stream >> row;
        } catch (std::exception &e) {
            std::cerr << "Map file is corrupted! (rows)" << std::endl;
            exit(1);
        }
        if (static_cast<int>(row.size()) != map_cols) {
            std::cerr << "Map file is corrupted! (cols)" << std::endl;
            exit(1);
        }
        for (int j = 0; j < map_cols; ++j) {
            switch (row[j]) {
                case '@':
                    map[i][j] = BLOCKED;
                    break;
                case '.':
                default:
                    break;
            }
        }
    }

    std::ifstream scenario_stream(scenario_file);
    if (!scenario_stream) {
        std::cerr << "Unable to open scenario file" << std::endl;
        exit(1);
    }

    int num_of_assigned, num_of_unassigned;
    scenario_stream >> num_of_assigned >> num_of_unassigned;

    vector<Location> assigned_starts(num_of_assigned), assigned_goals(num_of_assigned);
    for (int i = 0; i < num_of_assigned; ++i) {
        scenario_stream >> assigned_starts[i].y >> assigned_starts[i].x;
        scenario_stream >> assigned_goals[i].y >> assigned_goals[i].x;
    }

    vector<Location> unassigned_starts(num_of_unassigned);
    for (int i = 0; i < num_of_unassigned; ++i) {
        scenario_stream >> unassigned_starts[i].y >> unassigned_starts[i].x;
    }

    return std::make_shared<Scenario>(map_rows, map_cols, map, assigned_starts, assigned_goals, unassigned_starts);
}


int main(int argc, char** argv) {
    const auto vm = parse_args(argc, argv);
    const string map_file = vm["map"].as<std::string>();
    const string scenario_file = vm["scenario"].as<std::string>();
    const string solver_name = vm["alg"].as<std::string>();
    const int time_limit = vm["time"].as<int>();
    const string results_csv_file = vm["output"].as<std::string>();
    const int v_level = vm["v"].as<int>();

    std::ofstream out_csv;
    out_csv.open(results_csv_file, std::ios::app);
    if (std::filesystem::is_empty(results_csv_file)) {
        out_csv << "map_file;scen_file;cost;elapsed_time;unassigned_paths;assigned_paths" << std::endl << std::flush;
    }
    out_csv << std::fixed << std::setprecision(3) << std::flush;

    shared_ptr<Scenario> scenario = read_scenario(map_file, scenario_file);

    if (v_level) {
        std::cout << "Map: " << map_file << std::endl;
        std::cout << "Solver: " << solver_name << std::endl;
        std::cout << "Time limit: " << time_limit << " sec" << std::endl;
        std::cout << "Scenario: " << scenario_file << std::endl;
        std::cout << *scenario << std::endl;
    }

    std::stringstream ss;
    ss << std::fixed << std::setprecision(3);
    ss << std::filesystem::path(map_file).filename().string() << ";";
    ss << std::filesystem::path(scenario_file).filename().string() << ";";

    bool verbose = v_level > 1;
    bool time_limit_reached = false;
    Timer timer;
    timer.reset();
    shared_ptr<Plan> plan =
            (solver_name == "ACS") ? ACS(*scenario, verbose).solve(time_limit, time_limit_reached) :
            nullptr;
    timer.stop();
    auto elapsed_seconds = timer.elapsed_seconds();

    if (!plan) {

        if (time_limit_reached) {
            ss << "TLR";
            std::cout << "Time Limit Reached" << std::endl;
        } else {
            ss << "NF";
            std::cout << "Not Found" << std::endl;
        }
        ss << ";" << elapsed_seconds << ";;" << std::endl;
        out_csv << ss.str() << std::flush;
        return -1;
    }

    auto valid = plan->validate_plan(scenario->map_rows, scenario->map_cols, scenario->map,
                                     scenario->assigned_start_locations, scenario->assigned_goal_locations,
                                     scenario->unassigned_start_locations);

    if (!valid) {
        ss << "IVP;" << elapsed_seconds << ";;" << std::endl;
        std::cerr << "Invalid Plan" << std::endl;
        return -1;
    }
    if (v_level) {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << *plan << "Elapsed time: " << elapsed_seconds << " sec" << std::endl << std::endl;
    }

    ss << plan->get_makespan() << ";" << elapsed_seconds << ";";
    ss << "[";
    int num_assigned = static_cast<int>(plan->assigned_paths.size());
    for (int i = 0; i < num_assigned; ++i) {
        ss << *plan->assigned_paths.at(i);
        if (i < num_assigned - 1) ss << ",";
    }
    ss << "];[";
    int num_unassigned = static_cast<int>(plan->unassigned_paths.size());
    for (int i = 0; i < num_unassigned; ++i) {
        ss << *plan->unassigned_paths.at(i);
        if (i < num_unassigned - 1) ss << ",";
    }
    ss << "]" << std::endl;

    string out_str = ss.str();
    out_csv << out_str << std::flush;
    if (v_level == 0) {
        std::cout << out_str;
    }

    out_csv.close();
    return 0;
}
