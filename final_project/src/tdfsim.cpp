/**********************************************************************/
/*  Parallel-Fault Event-Driven Transition Delay Fault Simulator      */
/*                                                                    */
/*           Author: Tsai-Chieh Chen                                  */
/*           last update : 10/22/2018                                 */
/**********************************************************************/

#include "atpg.h"

/* pack 16 faults into one packet.  simulate 16 faults together. 
 * the following variable name is somewhat misleading */
#define num_of_pattern 16

/* The faulty_wire contains a list of wires that 
 * change their values in the fault simulation for a particular packet.
 * (that is wire_value1 != wire_value2) 
 * Note that the wire themselves are not necessarily a fault site.
 * The list is linked by the pnext pointers */

/* Had add*/
void ATPG::reverse_order_compression(int &total_detect_num) {//used in atpg.cpp
    string v1, v2;

    vector<string> vec = vectors;

    wptr w, faulty_wire;
    fptr simulated_fault_list[num_of_pattern];
    fptr f;
    int fault_type;
    int j, start_wire_index, nckt;
    int num_of_fault;
    int sum_of_detected = 0, num_of_detected;
    total_detect_num = 0;

    for (int i = vec.size()-1; i >=0; i--) {
        num_of_detected = 0;
        num_of_fault = 0;
        start_wire_index = 10000;

        //STEP1 : Assign V1
        vec[i].pop_back();//remove PI in timeframe2
        v1 = vec[i];//v1 pattern determine
    //cout<<"V1 = "<<v1<<endl;
        // start fault activation by v1
        for (int j = 0; j < cktin.size(); j++) {
            cktin[j]->value = ctoi(v1[j]);
        }

        nckt = sort_wlist.size();
        for (j = 0; j < nckt; j++) {
            if (j < cktin.size()) {
                sort_wlist[j]->set_changed();
            }
            else {
                sort_wlist[j]->value = U;
            }
        }

        sim();//Good sim for V1

        for (auto pos = flist_undetect.cbegin(); pos != flist_undetect.cend(); ++pos) {
            f = *pos;
            if (f->fault_type == sort_wlist[f->to_swlist]->value) {
                f->activate = 1;
            }
            else {
                f->activate = 0;
            }
        }
        // finish fault activation by v1

    //STEP2 : Shift in and get V2
        vec[i].pop_back();//shift
        v2 = vectors[i].back() + vec[i];//PI assign
    //cout<<"V2 = "<<v2<<endl;

        // start fsim by v2
        for (j = 0; j < cktin.size(); j++) {
            cktin[j]->value = ctoi(v2[j]);
        }

        nckt = sort_wlist.size();
        for (j = 0; j < nckt; j++) {
            if (j < cktin.size()) {
                sort_wlist[j]->set_changed();
            }
            else {
                sort_wlist[j]->value = U;
            }
        }

        sim();

        for (j = 0; j < nckt; j++) {
            switch (sort_wlist[j]->value) {
            case 1:
                sort_wlist[j]->wire_value1 = ALL_ONE;  // 11 represents logic one
                sort_wlist[j]->wire_value2 = ALL_ONE;
                break;
            case 2:
                sort_wlist[j]->wire_value1 = 0x55555555; // 01 represents unknown
                sort_wlist[j]->wire_value2 = 0x55555555;
                break;
            case 0:
                sort_wlist[j]->wire_value1 = ALL_ZERO; // 00 represents logic zero
                sort_wlist[j]->wire_value2 = ALL_ZERO;
                break;
            }
        } // for j

        //STEP3 Do a single stuck-at fault simulation
        /* walk through every undetected fault
        * the undetected fault list is linked by pnext_undetect */
        for (auto pos = flist_undetect.cbegin(); pos != flist_undetect.cend(); ++pos) {
            f = *pos;
            if (f->detect == REDUNDANT) { continue; } /* ignore redundant faults */
            /* consider only active (aka. excited) fault
            * (sa1 with correct output of 0 or sa0 with correct output of 1) */
            if (f->fault_type != sort_wlist[f->to_swlist]->value && f->activate) {
                /* if f is a primary output or is directly connected to an primary output
                * the fault is detected */
                if ((f->node->type == OUTPUT) ||
                    (f->io == GO && sort_wlist[f->to_swlist]->is_output())) {
                    f->detect = TRUE;
                }
                else {
                    /* if f is an gate output fault */
                    if (f->io == GO) {
                        //cout << "GO" << endl;
                        /* if this wire is not yet marked as faulty, mark the wire as faulty
                        * and insert the corresponding wire to the list of faulty wires. */
                        if (!(sort_wlist[f->to_swlist]->is_faulty())) {
                            sort_wlist[f->to_swlist]->set_faulty();
                            wlist_faulty.push_front(sort_wlist[f->to_swlist]);
                        }

                        /* add the fault to the simulated fault list and inject the fault */
                        simulated_fault_list[num_of_fault] = f;
                        inject_fault_value(sort_wlist[f->to_swlist], num_of_fault, f->fault_type);

                        /* mark the wire as having a fault injected
                        * and schedule the outputs of this gate */
                        sort_wlist[f->to_swlist]->set_fault_injected();
                        for (auto pos_n : sort_wlist[f->to_swlist]->onode) {
                            pos_n->owire.front()->set_scheduled();
                        }

                        /* increment the number of simulated faults in this packet */
                        num_of_fault++;
                        /* start_wire_index keeps track of the smallest level of fault in this packet.
                        * this saves simulation time.  */
                        start_wire_index = min(start_wire_index, f->to_swlist);
                    }  // if gate output fault
                    /* the fault is a gate input fault */
                    else {
                        /* if the fault is propagated, set faulty_wire equal to the faulty wire.
                        * faulty_wire is the gate output of f.  */
                        faulty_wire = get_faulty_wire(f, fault_type);
                        if (faulty_wire != nullptr) {

                            /* if the faulty_wire is a primary output, it is detected */
                            if (faulty_wire->is_output()) {
                                f->detect = TRUE;
                            }
                            else {
                                /* if faulty_wire is not already marked as faulty, mark it as faulty
                                * and add the wire to the list of faulty wires. */
                                if (!(faulty_wire->is_faulty())) {
                                    faulty_wire->set_faulty();
                                    wlist_faulty.push_front(faulty_wire);
                                }

                                /* add the fault to the simulated list and inject it */
                                simulated_fault_list[num_of_fault] = f;
                                inject_fault_value(faulty_wire, num_of_fault, fault_type);

                                /* mark the faulty_wire as having a fault injected
                                *  and schedule the outputs of this gate */
                                faulty_wire->set_fault_injected();
                                for (auto pos_n : faulty_wire->onode) {
                                    pos_n->owire.front()->set_scheduled();
                                }

                                num_of_fault++;
                                start_wire_index = min(start_wire_index, f->to_swlist);
                            }
                        }
                    }
                } // if  gate input fault
            } // if fault is active


            /*
            * fault simulation of a packet
            */

            /* if this packet is full (16 faults)
            * or there is no more undetected faults remaining (pos points to the final element of flist_undetect),
            * do the fault simulation */
            if ((num_of_fault == num_of_pattern) || (next(pos, 1) == flist_undetect.cend())) {
                //cout << "no more fault or full" << endl;
                /* starting with start_wire_index, evaulate all scheduled wires
                * start_wire_index helps to save time. */
                for (int j = start_wire_index; j < nckt; j++) {
                    if (sort_wlist[j]->is_scheduled()) {
                        sort_wlist[j]->remove_scheduled();
                        fault_sim_evaluate(sort_wlist[j]);
                    }
                } /* event evaluations end here */

                /* pop out all faulty wires from the wlist_faulty
                    * if PO's value is different from good PO's value, and it is not unknown
                    * then the fault is detected.
                    *
                    * IMPORTANT! remember to reset the wires' faulty values back to fault-free values.
                */
                while (!wlist_faulty.empty()) {
                    w = wlist_faulty.front();
                    wlist_faulty.pop_front();
                    w->remove_faulty();
                    w->remove_fault_injected();
                    w->set_fault_free();
                    /*TODO*/
                
                    /*
                    * After simulation is done,if wire is_output(), we should compare good value(wire_value1) and faulty value(wire_value2). 
                    * If these two values are different and they are not unknown, then the fault is detected.  We should update the simulated_fault_list.  Set detect to true if they are different.
                    * Since we use two-bit logic to simulate circuit, you can use Mask[] to perform bit-wise operation to get value of a specific bit.
                    * After that, don't forget to reset faulty values to their fault-free values.
                    */
                    if (w->is_output()) { // if primary output
                        for (j = 0; j < num_of_fault; j++) { // check every undetected fault
                            if (!(simulated_fault_list[j]->detect) && simulated_fault_list[j]->activate) {
                                if ((w->wire_value2 & Mask[j]) ^    // if value1 != value2
                                    (w->wire_value1 & Mask[j])) {
                                    if (((w->wire_value2 & Mask[j]) ^ Unknown[j]) &&  // and not unknowns
                                        ((w->wire_value1 & Mask[j]) ^ Unknown[j])) {
                                        simulated_fault_list[j]->detect = TRUE;  // then the fault is detected
                                    }
                                }
                            }
                        }
                    }
                    w->wire_value2 = w->wire_value1;  // reset to fault-free values
                    /*TODO*/
                } // pop out all faulty wires
                num_of_fault = 0;  // reset the counter of faults in a packet
                start_wire_index = 10000;  //reset this index to a very large value.
            } // end fault sim of a packet
        } // end loop. for f = flist

    //STEP4 : Drop faults have been detected 
        /* fault dropping  */
        flist_undetect.remove_if(
            [&](const fptr fptr_ele) {
                if (fptr_ele->detect == TRUE) {
                    sum_of_detected++;
                    num_of_detected++;
                    total_detect_num += fptr_ele->eqv_fault_num;
                    return true;
                }
                else {
                    return false;
                }
        });

        //cout << "#vector[" << i <<"] detects " << num_of_detected << " faults (" << sum_of_detected <<")" << endl;
        if (num_of_detected == 0) removable.push_back(1);
        else removable.push_back(0);
    }
    //display Hao
    int num_removed=0;
    for (int i = 0; i< atpg_result.size() ; ++i){
        if (!removable[atpg_result.size()-i-1]) cout<<atpg_result[i];
        else {//cout<<"# vector["<< i <<"] is deleted"<<endl ;
        ++num_removed;
        }
        //cout<<atpg_result[i];
    }
    cout << "#STC delete "<<num_removed<<" from "<<atpg_result.size()<<" patters; ";
    cout <<(num_removed*100.00)/atpg_result.size()<<" percent patterns reduce (higher better)"<<endl;
}





/* fault simulate a set of test vectors */

void ATPG::transition_delay_fault_simulation(int &total_detect_num) {
    string v1, v2;

    vector<string> vec = vectors;

    wptr w, faulty_wire;
    fptr simulated_fault_list[num_of_pattern];
    fptr f;
    int fault_type;
    int j, start_wire_index, nckt;
    int num_of_fault;
    int sum_of_detected = 0, num_of_detected;
    total_detect_num = 0;

    for (int i = vec.size()-1; i >=0; i--) {
        num_of_detected = 0;
	    num_of_fault = 0;
	    start_wire_index = 10000;

	    //STEP1 : Assign V1
	    vec[i].pop_back();//remove PI in timeframe2
        v1 = vec[i];//v1 pattern determine
	//cout<<"V1 = "<<v1<<endl;
        // start fault activation by v1
        for (int j = 0; j < cktin.size(); j++) {
            cktin[j]->value = ctoi(v1[j]);
        }

        nckt = sort_wlist.size();
        for (j = 0; j < nckt; j++) {
            if (j < cktin.size()) {
                sort_wlist[j]->set_changed();
            }
            else {
                sort_wlist[j]->value = U;
            }
        }

        sim();//Good sim for V1

        for (auto pos = flist_undetect.cbegin(); pos != flist_undetect.cend(); ++pos) {
            f = *pos;
            if (f->fault_type == sort_wlist[f->to_swlist]->value) {
                f->activate = 1;
            }
            else {
                f->activate = 0;
            }
        }
        // finish fault activation by v1

	//STEP2 : Shift in and get V2
        vec[i].pop_back();//shift
        v2 = vectors[i].back() + vec[i];//PI assign
	//cout<<"V2 = "<<v2<<endl;

        // start fsim by v2
        for (j = 0; j < cktin.size(); j++) {
            cktin[j]->value = ctoi(v2[j]);
        }

        nckt = sort_wlist.size();
        for (j = 0; j < nckt; j++) {
            if (j < cktin.size()) {
                sort_wlist[j]->set_changed();
            }
            else {
                sort_wlist[j]->value = U;
            }
        }

        sim();

        for (j = 0; j < nckt; j++) {
            switch (sort_wlist[j]->value) {
            case 1:
                sort_wlist[j]->wire_value1 = ALL_ONE;  // 11 represents logic one
                sort_wlist[j]->wire_value2 = ALL_ONE;
                break;
            case 2:
                sort_wlist[j]->wire_value1 = 0x55555555; // 01 represents unknown
                sort_wlist[j]->wire_value2 = 0x55555555;
                break;
            case 0:
                sort_wlist[j]->wire_value1 = ALL_ZERO; // 00 represents logic zero
                sort_wlist[j]->wire_value2 = ALL_ZERO;
                break;
            }
        } // for j

        //STEP3 Do a single stuck-at fault simulation
        /* walk through every undetected fault
        * the undetected fault list is linked by pnext_undetect */
        for (auto pos = flist_undetect.cbegin(); pos != flist_undetect.cend(); ++pos) {
            f = *pos;
            if (f->detect == REDUNDANT) { continue; } /* ignore redundant faults */
            /* consider only active (aka. excited) fault
            * (sa1 with correct output of 0 or sa0 with correct output of 1) */
            if (f->fault_type != sort_wlist[f->to_swlist]->value && f->activate) {
                /* if f is a primary output or is directly connected to an primary output
                * the fault is detected */
                if ((f->node->type == OUTPUT) ||
                    (f->io == GO && sort_wlist[f->to_swlist]->is_output())) {
                    f->detect = TRUE;
                }
                else {
                    /* if f is an gate output fault */
                    if (f->io == GO) {
                        //cout << "GO" << endl;
                        /* if this wire is not yet marked as faulty, mark the wire as faulty
                        * and insert the corresponding wire to the list of faulty wires. */
                        if (!(sort_wlist[f->to_swlist]->is_faulty())) {
                            sort_wlist[f->to_swlist]->set_faulty();
                            wlist_faulty.push_front(sort_wlist[f->to_swlist]);
                        }

                        /* add the fault to the simulated fault list and inject the fault */
                        simulated_fault_list[num_of_fault] = f;
                        inject_fault_value(sort_wlist[f->to_swlist], num_of_fault, f->fault_type);

                        /* mark the wire as having a fault injected
                        * and schedule the outputs of this gate */
                        sort_wlist[f->to_swlist]->set_fault_injected();
                        for (auto pos_n : sort_wlist[f->to_swlist]->onode) {
                            pos_n->owire.front()->set_scheduled();
                        }

                        /* increment the number of simulated faults in this packet */
                        num_of_fault++;
                        /* start_wire_index keeps track of the smallest level of fault in this packet.
                        * this saves simulation time.  */
                        start_wire_index = min(start_wire_index, f->to_swlist);
                    }  // if gate output fault
                    /* the fault is a gate input fault */
                    else {
                        /* if the fault is propagated, set faulty_wire equal to the faulty wire.
                        * faulty_wire is the gate output of f.  */
                        faulty_wire = get_faulty_wire(f, fault_type);
                        if (faulty_wire != nullptr) {

                            /* if the faulty_wire is a primary output, it is detected */
                            if (faulty_wire->is_output()) {
                                f->detect = TRUE;
                            }
                            else {
                                /* if faulty_wire is not already marked as faulty, mark it as faulty
                                * and add the wire to the list of faulty wires. */
                                if (!(faulty_wire->is_faulty())) {
                                    faulty_wire->set_faulty();
                                    wlist_faulty.push_front(faulty_wire);
                                }

                                /* add the fault to the simulated list and inject it */
                                simulated_fault_list[num_of_fault] = f;
                                inject_fault_value(faulty_wire, num_of_fault, fault_type);

                                /* mark the faulty_wire as having a fault injected
                                *  and schedule the outputs of this gate */
                                faulty_wire->set_fault_injected();
                                for (auto pos_n : faulty_wire->onode) {
                                    pos_n->owire.front()->set_scheduled();
                                }

                                num_of_fault++;
                                start_wire_index = min(start_wire_index, f->to_swlist);
                            }
                        }
                    }
                } // if  gate input fault
            } // if fault is active


            /*
            * fault simulation of a packet
            */

            /* if this packet is full (16 faults)
            * or there is no more undetected faults remaining (pos points to the final element of flist_undetect),
            * do the fault simulation */
            if ((num_of_fault == num_of_pattern) || (next(pos, 1) == flist_undetect.cend())) {
                //cout << "no more fault or full" << endl;
                /* starting with start_wire_index, evaulate all scheduled wires
                * start_wire_index helps to save time. */
                for (int j = start_wire_index; j < nckt; j++) {
                    if (sort_wlist[j]->is_scheduled()) {
                        sort_wlist[j]->remove_scheduled();
                        fault_sim_evaluate(sort_wlist[j]);
                    }
                } /* event evaluations end here */

                /* pop out all faulty wires from the wlist_faulty
                    * if PO's value is different from good PO's value, and it is not unknown
                    * then the fault is detected.
                    *
                    * IMPORTANT! remember to reset the wires' faulty values back to fault-free values.
                */
                while (!wlist_faulty.empty()) {
                    w = wlist_faulty.front();
                    wlist_faulty.pop_front();
                    w->remove_faulty();
                    w->remove_fault_injected();
                    w->set_fault_free();
                    /*TODO*/
                
                    /*
                    * After simulation is done,if wire is_output(), we should compare good value(wire_value1) and faulty value(wire_value2). 
                    * If these two values are different and they are not unknown, then the fault is detected.  We should update the simulated_fault_list.  Set detect to true if they are different.
                    * Since we use two-bit logic to simulate circuit, you can use Mask[] to perform bit-wise operation to get value of a specific bit.
                    * After that, don't forget to reset faulty values to their fault-free values.
                    */
                    if (w->is_output()) { // if primary output
                        for (j = 0; j < num_of_fault; j++) { // check every undetected fault
                            if (!(simulated_fault_list[j]->detect) && simulated_fault_list[j]->activate) {
                                if ((w->wire_value2 & Mask[j]) ^    // if value1 != value2
                                    (w->wire_value1 & Mask[j])) {
                                    if (((w->wire_value2 & Mask[j]) ^ Unknown[j]) &&  // and not unknowns
                                        ((w->wire_value1 & Mask[j]) ^ Unknown[j])) {
                                        simulated_fault_list[j]->detect = TRUE;  // then the fault is detected
                                    }
                                }
                            }
                        }
                    }
                    w->wire_value2 = w->wire_value1;  // reset to fault-free values
                    /*TODO*/
                } // pop out all faulty wires
                num_of_fault = 0;  // reset the counter of faults in a packet
                start_wire_index = 10000;  //reset this index to a very large value.
            } // end fault sim of a packet
        } // end loop. for f = flist

	//STEP4 : Drop faults have been detected 
        /* fault dropping  */
        flist_undetect.remove_if(
            [&](const fptr fptr_ele) {
                if (fptr_ele->detect == TRUE) {
                    sum_of_detected++;
                    num_of_detected++;
                    total_detect_num += fptr_ele->eqv_fault_num;
                    return true;
                }
                else {
                    return false;
                }
        });

        cout << "vector[" << i <<"] detects " << num_of_detected << " faults (" << sum_of_detected <<")" << endl;
    }
}


void ATPG::generate_tdfault_list() {//ref to init_flist.cpp
    int fault_num;
    wptr w;
    nptr n;
    fptr_s f;
    //unordered_map<wptr, int> num_of_eqv_sa0, num_of_eqv_sa1;
    /* walk through every wire in the circuit*/

    for (auto pos : sort_wlist) {
        w = pos;
        n = w->inode.front();

        /* for each gate, create a gate output STR fault */
        f = move(fptr_s(new(nothrow) FAULT));
        if (f == nullptr) error("No more room!");
        f->node = n;
        f->io = GO;     // gate output STR fault
        f->fault_type = STR;
        f->to_swlist = w->wlist_index;

        f->eqv_fault_num = 1;

        
        num_of_gate_fault += f->eqv_fault_num; // accumulate total uncollapsed faults
        flist_undetect.push_front(f.get()); // initial undetected fault list contains all faults
        flist.push_front(move(f));  // push into the fault list


        /* for each gate, create a gate output STF fault */
        f = move(fptr_s(new(nothrow) FAULT));
        if (f == nullptr) error("No more room!");
        f->node = n;
        f->io = GO;
        f->fault_type = STF;
        f->to_swlist = w->wlist_index;
        f->eqv_fault_num = 1;

        num_of_gate_fault += f->eqv_fault_num; // accumulate total fault count
        flist_undetect.push_front(f.get()); // initial undetected fault list contains all faults
        flist.push_front(move(f));  // push into the fault list


        /*if w has multiple fanout branches */
        if (w->onode.size() > 1) {
            for (nptr nptr_ele: w->onode) {
                /* create STR for each gate inputs  */
                f = move(fptr_s(new(nothrow) FAULT));
                if (f == nullptr) error("No more room!");
                f->node = nptr_ele;
                f->io = GI;
                f->fault_type = STR;
                f->to_swlist = w->wlist_index;
                f->eqv_fault_num = 1;
                /* f->index is the index number of gate input,
                    which GI fault is associated with*/
                for (int k = 0; k < nptr_ele->iwire.size(); k++) {
                    if (nptr_ele->iwire[k] == w) f->index = k;
                }
                num_of_gate_fault++;
                flist_undetect.push_front(f.get());
                flist.push_front(move(f));

                /* create STR for each gate inputs  */
                f = move(fptr_s(new(nothrow) FAULT));
                if (f == nullptr) error("No more room!");
                f->node = nptr_ele;
                f->io = GI;
                f->fault_type = STF;
                f->to_swlist = w->wlist_index;
                f->eqv_fault_num = 1;
                /* f->index is the index number of gate input,
                    which GI fault is associated with*/
                for (int k = 0; k < nptr_ele->iwire.size(); k++) {
                    if (nptr_ele->iwire[k] == w) f->index = k;
                }
                num_of_gate_fault++;
                flist_undetect.push_front(f.get());
                flist.push_front(move(f));
            }
        }
    }

    flist.reverse();
    flist_undetect.reverse();

    /*walk through all faults, assign fault_no one by one  */
    fault_num = 0;
    for (fptr f: flist_undetect) {
        f->fault_no = fault_num;
        fault_num++;
    }

    num_of_tdf_fault = fault_num;
}
