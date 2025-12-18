/*
 * Noxim - the NoC Simulator
 *
 * (C) 2005-2018 by the University of Catania
 * For the complete list of authors refer to file ../doc/AUTHORS.txt
 * For the license applied to these sources refer to file ../doc/LICENSE.txt
 *
 * This file contains the declaration of the Optical router
 */

#ifndef __NOXIMOPTIROUTER_H__
#define __NOXIMOPTIROUTER_H__

#include <systemc.h>
#include "DataStructs.h"
#include "Buffer.h"
#include "Stats.h"
#include "GlobalRoutingTable.h"
#include "LocalRoutingTable.h"
#include "ReservationTable.h"
#include "Utils.h"
#include "routingAlgorithms/RoutingAlgorithm.h"
#include "routingAlgorithms/RoutingAlgorithms.h"
#include "Power.h"
using namespace std;

extern unsigned int drained_volume;

SC_MODULE(OptiRouter)
{
    // I/O Ports
    sc_in_clk clock;		                  // The input clock for the router
    sc_in <bool> reset;                           // The reset signal for the router

    //ports to help in recieving and transmitting from electrical layer
    //recieving from electrical mesh
    sc_in <Flit> flit_electrical_rx;	  // The input channel
    sc_in <bool> req_electrical_rx;	  // The requests associated with the input channel
    sc_out <bool> ack_electrical_rx;	  // The outgoing ack signals associated with the input channel
    sc_out <TBufferFullStatus> buffer_full_status_rx;

    //transmitting to electrical mesh
    sc_out <Flit> flit_electrical_tx;   // The output channel
    sc_out <bool> req_electrical_tx;	  // The requests associated with the output channel
    sc_in <bool> ack_electrical_tx;	  // The outgoing ack signals associated with the output channel
    sc_in <TBufferFullStatus> buffer_full_status_tx;

    //port to recieve flits in optical layer
    sc_in <Flit> flit_optical_rx[DIRECTIONS];

    //ports to allow one hop transmission to all other optical routers
    //Will be array of ports, with index denoting destination id
    sc_out <Flit> *flit_optical_tx[DIRECTIONS];
    // sc_out <Flit> flit_optical_tx[DIRECTIONS+1];     
    // tried this but didn't work

    // Registers
    
    int local_id;		                  // Unique ID
    int routing_type;		              // Type of routing algorithm
    BufferBank buffer;		            // buffer only for incoming communication from electrical layer
    BufferBank buffer_opti;
    bool current_level_rx;	          // Current level for Alternating Bit Protocol (ABP) (for electrical)
    bool current_level_tx;	          // Current level for Alternating Bit Protocol (ABP) (for electrical)
    Stats stats;		                  // Statistics
    Power power;
    unsigned long routed_flits;
    RoutingAlgorithm * routingAlgorithm; 
    Flit last_received_flit;
    
    // Functions

    void process();
    void rxProcess();		// The receiving process
    void txProcess();		// The transmitting process
    void perCycleUpdate();
    void configure(const int _id, const double _warm_up_time,
		   const unsigned int _max_buffer_size);

    unsigned long getRoutedFlits();	// Returns the number of routed flits 

    //temporary functions to immitate control packet behaviour
    bool setReservationOptical(int src_id, int dst_id);
    void clearReservationOptical(int dst_id, int src_id);

    // Constructor

    SC_CTOR(OptiRouter) {
        
        //only need these systemC kernal process if optical flag is set
        if(GlobalParams::use_optinoc)
        {
            SC_METHOD(process);
            sensitive << reset;
            sensitive << clock.pos();

            SC_METHOD(perCycleUpdate);
            sensitive << reset;
            sensitive << clock.pos();
        }

        routingAlgorithm = RoutingAlgorithms::get(GlobalParams::routing_algorithm);

        if (routingAlgorithm == 0)
        {
            cerr << " FATAL: invalid routing -routing " << GlobalParams::routing_algorithm << ", check with noxim -help" << endl;
            exit(-1);
        }
    }

  private:


  public:
    unsigned int local_drained;

    void ShowBuffersStats(std::ostream & out);
};

#endif
