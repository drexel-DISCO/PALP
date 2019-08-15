#include <iostream>
#include <fstream>

#include "Simulation.h"

void FullSystemSimulation(Config &cfg,
                          std::vector<std::string> &trace_lists,
                          std::string stats_output_file);

int main(int argc, const char *argv[])
{
    auto [cfg_file,
          trace_lists,
          stats_output_file ] = parse_args(argc, argv);
    assert(trace_lists.size() != 0);
    
    std::cout << "\nConfiguration file: " << cfg_file << "\n";
    std::cout << "Stats output file: " << stats_output_file << "\n";

    Config cfg(cfg_file);
    FullSystemSimulation(cfg,
                         trace_lists,
                         stats_output_file);
}

void FullSystemSimulation(Config &cfg,
                          std::vector<std::string> &trace_lists,
                          std::string stats_output_file)
{
    unsigned num_of_cores = trace_lists.size();
    
    /* Memory System Creation */
    // Create (PCM) main memory
    std::unique_ptr<MemObject> PCM(createMemObject(cfg, Memories::PCM));

    // Create eDRAM
//    std::unique_ptr<MemObject> eDRAM(createMemObject(cfg, Memories::eDRAM, isLLC));
//    eDRAM->setNextLevel(PCM.get());

    // Create L2
//    std::unique_ptr<MemObject> L2(createMemObject(cfg, Memories::L2_CACHE, isNonLLC));
    std::unique_ptr<MemObject> L2(createMemObject(cfg, Memories::L2_CACHE, isLLC));
//    L2->setNextLevel(eDRAM.get());
    L2->setNextLevel(PCM.get());
    L2->setArbitrator(num_of_cores);

    // Create private L1-D-Cache
    std::vector<std::unique_ptr<MemObject>> L1_D_all;
    for (int i = 0; i < num_of_cores; i++)
    {
        // Create L1-D
        std::unique_ptr<MemObject> L1_D(createMemObject(cfg, Memories::L1_D_CACHE, isNonLLC));
        L1_D->setId(i);
        L1_D->setBoundaryMemObject();
        L1_D->setNextLevel(L2.get());

        L1_D_all.push_back(std::move(L1_D));
    }
    // Create MMU
    std::unique_ptr<System::MMU> mmu = std::make_unique<System::MMU> (num_of_cores);
    
    // Create Processor 
    std::unique_ptr<Processor> processor(new Processor(trace_lists, L2.get()));
    processor->setMMU(mmu.get());
    for (int i = 0; i < num_of_cores; i++) 
    {
        processor->setDCache(i, L1_D_all[i].get());
    }
   
    std::cout << "\nSimulation Stage...\n\n";
    runCPUTrace(processor.get());
    
    /* Collecting Stats */
    Stats stats;

    for (auto &L1_D : L1_D_all)
    {
        L1_D->registerStats(stats);
    }
    L2->registerStats(stats);
//    eDRAM->registerStats(stats);
    PCM->registerStats(stats);
    stats.registerStats("Execution Time (cycles) = " + 
                        std::to_string(processor->exeTime()));
    stats.outputStats(stats_output_file);
}
