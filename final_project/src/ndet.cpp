#include "atpg.h"


void ATPG::ndet_test() {
    if (tdfsim_only)
    {
        // Transition delay fault simulation
        int total_detect_num = 0;
        flist_undetect.clear();
        flist.clear();
        generate_tdfault_list();
        transition_delay_fault_simulation(total_detect_num);
        in_vector_no += vectors.size();
        display_undetect();

        printf("\n# Result:\n");
        printf("-----------------------\n");
        printf("# total transition delay faults: %d\n", num_of_tdf_fault);
        printf("# total detected faults: %d\n", total_detect_num);
        printf("# fault coverage: %lf %\n", (double) total_detect_num / (double) num_of_tdf_fault * 100);
        return;
    }

    int max_detected_num = detected_num;
    int n = 1;
/*
4.  For K=1 to (N-1)
    -   Perform multiple-detect fault simulation
        with pattern sets T1 to TK for TF MD faults
    -   Save faults detected K times (FK)
    -   Target faults FK and perform single-detect
        ATPG to increase the number of detections
        by one
    -   Save the patterns to T(K+1)
*/
    
    

    while (n <= max_detected_num) {
        // generate single-detect pattern
        detected_num = 1;
        test();
        // tdfsim
        set_tdfsim_only(true);
        detected_num = n;
        ndet_test();
        set_tdfsim_only(false);
        std::forward_list<fptr> fault_detected;
        fptr f;
        
        for (const auto &pos : flist) {
            f = pos.get();
            if (f->detect == TRUE) {
                fault_detected.push_front(f);
                f->detect = FALSE;
            }
        }
        
        flist_undetect = fault_detected;

        timer(stdout, "for running ATPG");
        cout << "Detecting " << n << "\n";
        ++n;
    }


/*
5.  Perform multiple-detect fault simulation with
    pattern sets T1 to TN for all faults to obtain
    multiple-detect fault coverage profile
*/
/*
    for (pattern : patterns) {
        tdfault_sim_a_vector());    
    }
    */


}
