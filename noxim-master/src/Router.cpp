/*
 * Noxim - the NoC Simulator
 *
 * (C) 2005-2018 by the University of Catania
 * For the complete list of authors refer to file ../doc/AUTHORS.txt
 * For the license applied to these sources refer to file ../doc/LICENSE.txt
 *
 * This file contains the implementation of the router
 */

#include "Router.h"
#define RCUmul 1.8
#define convConst 0.2
//0.11 + 0.09 pJ
int flit_src_id_trace=0;
int flit_dst_id_trace=0;
bool first_flit=false;
double req_ack_start_time=0.0;//sc_time_stamp().to_double()
double req_ack_end_time=0.0;
double head_tail_start_time=0.0;
double head_tail_end_time=0.0;
double destroy_start_time=0.0;
double destroy_end_time=0.0;
Flit flit1;
double global_numOnStateMR = 0.0;
double rcuCost = 0.0;
double EtoOPCost = 0.0;
// int number_of_req_ack_etc_captured=0;

inline int toggleKthBit(int n, int k)
{
	return (n ^ (1 << (k-1)));
}

void Router::process()
{	
	
    txProcess();
    rxProcess();
}

void Router::rxProcess()
{
    if (reset.read()) {
	TBufferFullStatus bfs;
	// Clear outputs and indexes of receiving protocol
	for (int i = 0; i < DIRECTIONS + 2; i++) {
	    ack_rx[i].write(0);
	    current_level_rx[i] = 0;
	    buffer_full_status_rx[i].write(bfs);
	}
	routed_flits = 0;
	local_drained = 0;
    } 
	
    else 
    { 
	// This process simply sees a flow of incoming flits. All arbitration
	// and wormhole related issues are addressed in the txProcess()
	//assert(false);
	for (int i = 0; i < DIRECTIONS + 2; i++) {
	    // To accept a new flit, the following conditions must match:
	    // 1) there is an incoming request
	    // 2) there is a free slot in the input buffer of direction i
	    //LOG<<"****RX****DIRECTION ="<<i<<  endl;

	    if (req_rx[i].read() == 1 - current_level_rx[i])
	    { 
		Flit received_flit = flit_rx[i].read();
		//LOG<<"request opposite to the current_level, reading flit "<<received_flit<<endl;

		int vc = received_flit.vc_id;

		flit_dst_id_trace=received_flit.dst_id;
		flit_src_id_trace=received_flit.src_id;

		//NEW, Whenever the recieved flit is ACK control flit, push it into buffer_ack instead of normal buffer
		if(received_flit.flit_type==FLIT_TYPE_ACK)
		{
			buffer_ack[vc].Push(received_flit);
			LOG << " Flit " << received_flit << " collected from Input[" << i << "][" << vc <<"]" << endl;
			power.bufferRouterPush();
			current_level_rx[i] = 1 - current_level_rx[i];
			ack_rx[i].write(current_level_rx[i]);
			continue;
		}
		// //----------------------------------------------------
		// LOG<<flit_src_id_trace<<endl;
		// LOG<<flit_dst_id_trace<<endl;

		if (!buffer[i][vc].IsFull()) 
		{

		    // Store the incoming flit in the circular buffer
		    buffer[i][vc].Push(received_flit);
			//logging
			// if(i==DIRECTION_HUB)
			// 	cout<<local_id<<" PUSHED:"<<received_flit<<endl;

		    LOG << " Flit " << received_flit << " collected from Input[" << i << "][" << vc <<"]" << endl;

		    power.bufferRouterPush();

		    // Negate the old value for Alternating Bit Protocol (ABP)
		    //LOG<<"INVERTING CL FROM "<< current_level_rx[i]<< " TO "<<  1 - current_level_rx[i]<<endl;
		    current_level_rx[i] = 1 - current_level_rx[i];

		    // if a new flit is injected from local PE
		    if (received_flit.src_id == local_id)
			power.networkInterface();

			//NEW. IF optical mesh enabled and recieved flit is from optical mesh, O/E conversion took place
			//power for that needs to be added
			if(GlobalParams::use_optinoc && i==DIRECTION_HUB)
			{
				power.opticalInterface();
			}
		}

		else  // buffer full
		{
		    // should not happen with the new TBufferFullStatus control signals    
		    // except for flit coming from local PE, which don't use it 
		    LOG << " Flit " << received_flit << " buffer full Input[" << i << "][" << vc <<"]" << endl;
		    assert(i== DIRECTION_LOCAL);
		}

	    }
			
	    ack_rx[i].write(current_level_rx[i]);
	    // updates the mask of VCs to prevent incoming data on full buffers
	    TBufferFullStatus bfs;
	    for (int vc=0;vc<GlobalParams::n_virtual_channels;vc++)
		bfs.mask[vc] = buffer[i][vc].IsFull();
	    buffer_full_status_rx[i].write(bfs);

		// if(buffer[DIRECTION_HUB][0].IsFull())
		// 	cout<<"DIRECTION HUB router buffer being full"<<endl;
	}
    }
}

void Router::txProcess()
{

	if (reset.read()) 
	{
      	// Clear outputs and indexes of transmitting protocol
      	for (int i = 0; i < DIRECTIONS + 2; i++) 
		{
	  		req_tx[i].write(0);
	  		current_level_tx[i] = 0;
		}

		//reset data related to control packets for optical communication
		for(int i=0;i<DIRECTIONS+1;i++)
			opticalReserve[i]=false;
		
		reqSent=false;
		ackReceived=false;
		numOnStateMR=0;
    } 
  	else 
    { 

		//always check if there is any destroy flit to be sent now first
		if(GlobalParams::use_optinoc && !desFlitQ.empty())
		{
			
			Flit desFlit=desFlitQ.front().first;
			int targetCycle=desFlitQ.front().second;
			int current_sim_cycles = sc_time_stamp().to_double()/GlobalParams::clock_period_ps;
			int vc=desFlit.vc_id;

			//check if targetTime of destroy flit reached
			if(current_sim_cycles>targetCycle)
			{
				//attempt to send the destroy flit (X-Y routing)
				int o;
				Coord current = id2Coord(local_id);
				Coord destination = id2Coord(desFlit.dst_id);

				if(destination.x>current.x)
					o=DIRECTION_EAST;
				else if(destination.x<current.x)
					o=DIRECTION_WEST;
				else if(destination.y>current.y)
					o=DIRECTION_SOUTH;
				else
					o=DIRECTION_NORTH;

				//check if destroy flit can be sent on the output port
				if ( (current_level_tx[o] == ack_tx[o].read()) &&
					(buffer_full_status_tx[o].read().mask[vc] == false) ) 
				{
					//clear output direction of optical router
					opticalReserve[o]=false;

					LOG << " cleared direction "<< o << " of optical router:" << local_id << " by flit:" << desFlit << endl;
					LOG << " DES flit forwared to output["<<o<<"], flit:"<<desFlit<<endl;
					
					flit_tx[o].write(desFlit);
					current_level_tx[o] = 1 - current_level_tx[o];
					req_tx[o].write(current_level_tx[o]);
					
					//pop the top entry from desFlitQ since 
					desFlitQ.pop();

					//Destroy flit sent from source router. MR at source needs to be switched off
					numOnStateMR--;
				}
			}

			//skip current transmission cycle if there is some destroy flit which wasnt sent in the queue
			//all other flits will be processed only after the sending the destroy flit
			return;
		}

		//NEW, before looking at normal buffers before transmission, we look at buffer_ack
		//and transmit any ack control flit first. This is done to prevent deadlocking
		if(GlobalParams::use_optinoc)
		{
			for (int vc = 0;vc < GlobalParams::n_virtual_channels; vc++)
			{
				if(!buffer_ack[vc].IsEmpty())
				{
					Flit flit = buffer_ack[vc].Front();
					power.bufferRouterFront();
					flit.hop_no++;
					//Start of garbage. Send ACK flit to corresponding destination
					int src_id = local_id;
					int dst_id = flit.dst_id;
					
					//special case when ACK flit reaches destination
					if(dst_id==local_id)
					{
						//ACK flit reached destination, make ackReceived boolean true and pop the flit
						
						// if(flit.src_id==flit1.dst_id && flit.dst_id==flit1.src_id && flit.vc_id==flit1.vc_id){
						// 	req_ack_end_time=sc_time_stamp().to_double()/GlobalParams::clock_period_ps;
						// 	cout<<"Time taken for the request and ack flit to reach destination:"<<req_ack_end_time-req_ack_start_time<<endl;
						// 	cout<<"flit_timestamp : "<<flit.timestamp<<" flit1_timestamp : "<<flit1.timestamp<<endl;
						// }
						// cout<<sc_time_stamp().to_double()<<endl;
						// yaha pe ack ka end time dekhna -------------------------
						ackReceived = true;
						//new
						stats.receivedFlit(sc_time_stamp().to_double() / GlobalParams::clock_period_ps, flit);
						
						LOG << " Ack Recieved at final destination:" << local_id <<endl;
						//pop the ack flit from the buffer
						buffer_ack[vc].Pop();
						power.bufferRouterPop();
						
						//skip current iteration as all operations related to top flit completed and popped
						continue;
					}

					//reached here means control packet destination is different
					int o;

					Coord current = id2Coord(src_id);
					Coord destination = id2Coord(dst_id);

					//X-Y routing to find output direction
					if(destination.x>current.x)
						o=DIRECTION_EAST;
					else if(destination.x<current.x)
						o=DIRECTION_WEST;
					else if(destination.y>current.y)
						o=DIRECTION_SOUTH;
					else
						o=DIRECTION_NORTH;
					
					//check if possible to forward
					//something to do with this for optical mesh
					if ( current_level_tx[o] == ack_tx[o].read() )
					{
						LOG << "ACK flit forwarded to Output[" << o << "], flit: " << flit << endl;

						flit_tx[o].write(flit);
						current_level_tx[o] = 1 - current_level_tx[o];
						req_tx[o].write(current_level_tx[o]);
						buffer_ack[vc].Pop();
						
						power.r2rLink();
						power.bufferRouterPop();
						power.crossBar();
					}
					else
					{
						LOG << " Cannot forward ACK flit to Output[" << o << "], flit: " << flit << endl;
					}
				}
			}
		}

    // 1st phase: Reservation . For optical, forwarding is done instantly
    for (int j = 0; j < DIRECTIONS + 2; j++) 
	{
		int i = (start_from_port + j) % (DIRECTIONS + 2);

	  	for (int k = 0;k < GlobalParams::n_virtual_channels; k++)
	  	{
			int vc = (start_from_vc[i]+k)%(GlobalParams::n_virtual_channels);
	      
	      	// Uncomment to enable deadlock checking on buffers. 
	      	// Please also set the appropriate threshold.
	      	// buffer[i].deadlockCheck();

	      	if (!buffer[i][vc].IsEmpty()) 
	      	{
		  	Flit flit = buffer[i][vc].Front();
		  	power.bufferRouterFront();
			
			//handle functionality of router when optical mesh enabled
			if(GlobalParams::use_optinoc)
			{
				//head and tail flit coming from local direction need the use of control packets
				
				//head flit from local direction
				if(i==DIRECTION_LOCAL && flit.flit_type == FLIT_TYPE_HEAD)
				{
					//check if ack recieved, if so try to forward the head flit to hub
					//If ack not recieved check if req has been sent. 
					//if not sent the request and wait for ack
					if(ackReceived)
					{
						//attempt to forward the head flit
						// if(flit.isEquivalent(flit1)){
						// 	head_tail_start_time=sc_time_stamp().to_double()/GlobalParams::clock_period_ps;
						// 	cout<<"head_tail_start_time : "<<head_tail_start_time<<endl;
						// 	cout<<"flit_timestamp : "<<flit.timestamp<<" flit1_timestamp : "<<flit1.timestamp<<endl;
						// }
						int o=DIRECTION_HUB;

						LOG << "ACK flit was received and head flit:"<< flit << "can be forwarded to optical mesh" << endl;

						if ( (current_level_tx[o] == ack_tx[o].read()) &&
							(buffer_full_status_tx[o].read().mask[vc] == false) ) 
						{
							LOG << "Input[" << i << "][" << vc << "] forwarded to Output[" << o << "], flit: " << flit << endl;

							flit_tx[o].write(flit);
							current_level_tx[o] = 1 - current_level_tx[o];
							req_tx[o].write(current_level_tx[o]);
							buffer[i][vc].Pop();
							
							//clear the bool values corresponding to head flit
							ackReceived=false;
						 	reqSent=false;


							/* Power & Stats ------------------------------------------------- */
							if (o == DIRECTION_HUB) 
								power.r2hLink();
							else
								power.r2rLink();

							power.bufferRouterPop();
							power.crossBar();

							//Head flit is sent to Optical mesh, so add power for E/O conversion
							power.opticalInterface();
						}
						else
						{
							LOG << " Cannot forward Input[" << i << "][" << vc << "] to Output[" << o << "], flit: " << flit << endl;
							LOG << " **DEBUG buffer_full_status_tx " << buffer_full_status_tx[o].read().mask[vc] << endl;
						}

					}
					//if request for head flit not sent, send it
					else if(!reqSent)
					{
						// Yaha pe time check start karna hai request flit ka --------------------------------------
						if(first_flit==false)
						{
							first_flit=true;				
							flit1=flit;
							// number_of_req_ack_etc_captured+=1;
						}
						
						LOG<< "Attempting to send REQ flit for the head flit:" << flit << endl;
						
						// if(flit1.isEquivalent(flit)){
						// 	req_ack_start_time=sc_time_stamp().to_double()/GlobalParams::clock_period_ps;
						// 	cout<<"req_ack_start_time : "<<req_ack_start_time<<endl;
						// 	cout<<"flit_timestamp : "<<flit.timestamp<<" flit1_timestamp : "<<flit1.timestamp<<endl;
						// }
						//nandan
						// rcuCost+=(hops+1)*RCUmul*num_packets_sent;
						// EtoOPCost+=(convConst)*num_packets_sent;
						//the request flit would be exactly same as head flit except for type
						flit.flit_type = FLIT_TYPE_REQ;
						// flit.timestamp=sc_time_stamp().to_double()/GlobalParams::clock_period_ps;
						//now attempt to send it in the required direction (X-Y routing)
						int o;
						Coord current = id2Coord(local_id);
						Coord destination = id2Coord(flit.dst_id);

						if(destination.x>current.x)
							o=DIRECTION_EAST;
						else if(destination.x<current.x)
							o=DIRECTION_WEST;
						else if(destination.y>current.y)
							o=DIRECTION_SOUTH;
						else
							o=DIRECTION_NORTH;

						//check if optical direction can be reserved
						if(!opticalReserve[o])
						{
							//check if req flit can be sent on the output port
							if ( (current_level_tx[o] == ack_tx[o].read()) &&
								(buffer_full_status_tx[o].read().mask[vc] == false) ) 
							{
								//reserve output direction of optical router
								opticalReserve[o]=true;
								
								LOG << " Reserved direction "<< o << " of optical router:" << local_id << " by flit:" << flit << endl;
								LOG << "REQ FLIT forwarded to Output[" << o << "], flit: " << flit << endl;
								
								flit_tx[o].write(flit);
								current_level_tx[o] = 1 - current_level_tx[o];
								req_tx[o].write(current_level_tx[o]);
								
								reqSent=true;

								//when REQ flit is created, that is source router. MR needs to be switched on
								numOnStateMR++;
								global_numOnStateMR++;
								
							}
						}
					}
					//if req was sent and ack not recieved, do nothing
					continue;
				}
				//tail flit coming from local direction
				//create destroy flit and then put it into queue to forward after 2 cycles
				else if(i==DIRECTION_LOCAL && flit.flit_type == FLIT_TYPE_TAIL)
				{
					//first send the tail flit
					int o=DIRECTION_HUB;

					if ( (current_level_tx[o] == ack_tx[o].read()) &&
							(buffer_full_status_tx[o].read().mask[vc] == false) ) 
					{
						//if tail flit is same as head flit, then store the time when head and tail flit reached destination
						// if(flit.isEquivalent(flit1)){
						// 	head_tail_end_time=sc_time_stamp().to_double()/GlobalParams::clock_period_ps;
						// 	cout<<"Time taken for the head and tail flit to reach destination:"<<head_tail_end_time-head_tail_start_time<<endl;
						// 	cout<<"flit_timestamp : "<<flit.timestamp<<" flit1_timestamp : "<<flit1.timestamp<<endl;
						// }
						LOG << "Input[" << i << "][" << vc << "] forwarded to Output[" << o << "], flit: " << flit << endl;

						flit_tx[o].write(flit);
						current_level_tx[o] = 1 - current_level_tx[o];
						req_tx[o].write(current_level_tx[o]);
						buffer[i][vc].Pop();
						
						//if first flit, then store the time when head and tail flit reached destination
						//the destroy flit will be same as the tail flit except for the type
						flit.flit_type = FLIT_TYPE_DES;
						flit.timestamp=sc_time_stamp().to_double()/GlobalParams::clock_period_ps;
						
						//push the current destroy flit into queue along with time
						int current_sim_cycles = sc_time_stamp().to_double()/GlobalParams::clock_period_ps;
						int targetTime=current_sim_cycles+2;
						LOG << "Destroy control flit:" << flit << " pushed into queue with target time:" << targetTime << endl;
						// if(flit.isEquivalent(flit1)){
						// 	destroy_start_time=sc_time_stamp().to_double()/GlobalParams::clock_period_ps;
						// 	cout<<"destroy_start_time : "<<destroy_start_time<<endl;
						// 	cout<<"flit_timestamp : "<<flit.timestamp<<" flit1_timestamp : "<<flit1.timestamp<<endl;
						// }
						//now add this flit into the queue
						desFlitQ.push(make_pair(flit,targetTime));

						//}

						//reverting the flit to original type
						flit.flit_type = FLIT_TYPE_TAIL;


						/* Power & Stats ------------------------------------------------- */
						if (o == DIRECTION_HUB) 
							power.r2hLink();
						else
							power.r2rLink();

						power.bufferRouterPop();
						power.crossBar();

						//TAIL flit sent to Optical mesh, so E/O conversion took place. Power needs to be added
						power.opticalInterface();
					}
					else
					{
						LOG << " Cannot forward Input[" << i << "][" << vc << "] to Output[" << o << "], flit: " << flit << endl;
						LOG << " **DEBUG buffer_full_status_tx " << buffer_full_status_tx[o].read().mask[vc] << endl;
					}
				}
				//handle all control packets
				else if (flit.flit_type == FLIT_TYPE_REQ || flit.flit_type == FLIT_TYPE_ACK || flit.flit_type == FLIT_TYPE_DES) 
				{
					int src_id = local_id;
					int dst_id = flit.dst_id;
					
					//special case when control flit reaches destination
					if(dst_id==local_id)
					{
						//req flit reached destination, check if local direction is free and ACK can be sent back
						if(flit.flit_type == FLIT_TYPE_REQ)
						{
							//check if local direction is free
							if(!opticalReserve[DIRECTION_LOCAL])
							{							
								//do x-y routing to find direction to send ACK flit
								int o;
								Coord current = id2Coord(local_id);
								Coord destination = id2Coord(flit.src_id);

								if(destination.x>current.x)
									o=DIRECTION_EAST;
								else if(destination.x<current.x)
									o=DIRECTION_WEST;
								else if(destination.y>current.y)
									o=DIRECTION_SOUTH;
								else
									o=DIRECTION_NORTH;

								//check if ack flit can be sent on the output port
								if ( (current_level_tx[o] == ack_tx[o].read()) &&
									(buffer_full_status_tx[o].read().mask[vc] == false) ) 
								{
									//reserve local direction of destination optical router
									opticalReserve[DIRECTION_LOCAL]=true;
									LOG << " reserved direction " << DIRECTION_LOCAL << " of optical router:" << local_id << " by flit:" << flit << endl;

									//change the current flit into ACK flit by switching src and destination
									flit.dst_id=flit.src_id;
									flit.src_id=local_id;
									flit.flit_type=FLIT_TYPE_ACK;

									// ack flit ka shuru yaha se karna -------------------------------------

									LOG << "ACK flit created at:"<< local_id << " and forwarded to Output[" << o << "], flit: " << flit << endl;

									flit_tx[o].write(flit);
									current_level_tx[o] = 1 - current_level_tx[o];
									req_tx[o].write(current_level_tx[o]);
									
									//pop the REQ flit now that everything is completed
									buffer[i][vc].Pop();
									power.bufferRouterPop();

									//MR switched on at the destination optical router
									numOnStateMR++;
									global_numOnStateMR++;
								}
								else
								{
									LOG << " Cannot forward Input[" << i << "][" << vc << "] to Output[" << o << "], flit: " << flit << endl;
								}
							}
							else
							{
								LOG << "direction 4 of optical router:" << local_id << "is blocked. REQ flit:"<<flit<<"cannot be forwarded" << endl;
							}
						}
						//ACK flit reached destination, make ackReceived boolean true and pop the flit
						else if(flit.flit_type == FLIT_TYPE_ACK)
						{
							//make ackReceived boolean to be true as ack recieved at the destination
							ackReceived = true;
							
							// if(flit.src_id==flit1.dst_id && flit.dst_id==flit1.src_id && flit.vc_id==flit1.vc_id){
							// 	req_ack_end_time=sc_time_stamp().to_double()/GlobalParams::clock_period_ps;
							// 	cout<<"Time taken for the request and ack flit to reach destination:"<<req_ack_end_time-req_ack_start_time<<endl;
							// 	cout<<"flit_timestamp : "<<flit.timestamp<<" flit1_timestamp : "<<flit1.timestamp<<endl;
							// }
							// yaha pe ack ka end time dekhna -------------------------
							
							//new
							stats.receivedFlit(sc_time_stamp().to_double() / GlobalParams::clock_period_ps, flit);
							
							LOG << " Ack Recieved at final destination:" << local_id <<endl;
							//pop the ack flit from the buffer
							buffer[i][vc].Pop();
							power.bufferRouterPop();
						}
						//destroy control flit reached destination
						else
						{
							
							// if(flit.isEquivalent(flit1)){
							// 	destroy_end_time=sc_time_stamp().to_double()/GlobalParams::clock_period_ps;
							// 	cout<<"Time taken for the destroy flit to reach destination:"<<destroy_end_time-head_tail_end_time<<endl;
							// 	cout<<"flit_timestamp : "<<flit.timestamp<<" flit1_timestamp : "<<flit1.timestamp<<endl;
							// }
							
							// new -- tracking the destroy time
							stats.receivedFlit(sc_time_stamp().to_double() / GlobalParams::clock_period_ps, flit);
							
							opticalReserve[DIRECTION_LOCAL]=false;
							LOG << " cleared direction "<< DIRECTION_LOCAL << " of optical router:" << local_id << " by flit:" << flit << endl;
							//pop the destroy flit from the buffer
							
							buffer[i][vc].Pop();
							power.bufferRouterPop();

							//Destroy flit reached destination. The MR at destination is to be switched off
							numOnStateMR--;
						}
						//skip current iteration as all operations related to top flit completed and popped
						continue;
					}

					//reached here means control packet destination is different
					int o;

					Coord current = id2Coord(src_id);
					Coord destination = id2Coord(dst_id);

					//X-Y routing to find output direction
					if(destination.x>current.x)
						o=DIRECTION_EAST;
					else if(destination.x<current.x)
						o=DIRECTION_WEST;
					else if(destination.y>current.y)
						o=DIRECTION_SOUTH;
					else
						o=DIRECTION_NORTH;
					
					//check if possible to forward
					if ( (current_level_tx[o] == ack_tx[o].read()) &&
							(buffer_full_status_tx[o].read().mask[vc] == false) ) 
					{
						//if req flit, check if output port is free, and if free make it blocked then forward flit
						if(flit.flit_type==FLIT_TYPE_REQ)
						{
							//checking if output port is free, if not we won't forward this flit
							if(!opticalReserve[o])
							{
								opticalReserve[o]=true;
								LOG << " reserved direction "<< o << " of optical router:" << local_id << " by flit:" << flit << endl;
							}
							else
							{
								LOG << " direction " << o << " of optical router:"<<local_id << "is blocked. REQ flit:"<< flit << "cannot be forwarded" << endl;
								continue;
							}
						}
						//if destroy flit, then make the output direction free and then forward flit
						else if(flit.flit_type==FLIT_TYPE_DES)
						{
							opticalReserve[o]=false;
							LOG << " cleared direction "<< o << " of optical router:" << local_id << " by flit:" << flit << endl;
						}

						LOG << "Input[" << i << "][" << vc << "] forwarded to Output[" << o << "], flit: " << flit << endl;

						flit_tx[o].write(flit);
						current_level_tx[o] = 1 - current_level_tx[o];
						req_tx[o].write(current_level_tx[o]);
						buffer[i][vc].Pop();
						
						power.r2rLink();
						power.bufferRouterPop();
						power.crossBar();

						//check if this forwarding for REQ flit required a change in direction, if so 
						//increase number of on-state MRs
						if(flit.flit_type == FLIT_TYPE_REQ)
						{
							if((i==DIRECTION_NORTH && o!=DIRECTION_SOUTH) || 
								(i==DIRECTION_EAST && o!=DIRECTION_WEST)  || 
								(i==DIRECTION_SOUTH && o!=DIRECTION_NORTH) || 
								(i==DIRECTION_WEST && o!=DIRECTION_EAST))
							{
								numOnStateMR++;
								global_numOnStateMR++;
							}

						}

						//check if forwarding of destroy flit resulted in change of direction, if so
						//this made an MR switch on. We revert that MR to off state
						if(flit.flit_type == FLIT_TYPE_DES)
						{
							if((i==DIRECTION_NORTH && o!=DIRECTION_SOUTH) || 
								(i==DIRECTION_EAST && o!=DIRECTION_WEST)  || 
								(i==DIRECTION_SOUTH && o!=DIRECTION_NORTH) || 
								(i==DIRECTION_WEST && o!=DIRECTION_EAST))
							{
								numOnStateMR--;
							}
						}
					}
					else
					{
						LOG << " Cannot forward Input[" << i << "][" << vc << "] to Output[" << o << "], flit: " << flit << endl;
						LOG << " **DEBUG buffer_full_status_tx " << buffer_full_status_tx[o].read().mask[vc] << endl;
					}

					
				}
				//packets which are not control packets and not head or tail flits coming from local direction
				//send the packets coming from DIRECTION_LOCAL to DIRECTION_HUB and vice versa
				else
				{
					int o;
					if(i==DIRECTION_LOCAL)
						o=DIRECTION_HUB;
					else if(i==DIRECTION_HUB)
						o=DIRECTION_LOCAL;
					//shud not be happeneing
					else
					{
						LOG<<"ERROR OTher type packet reached final else part: "<<flit<<endl;
						assert(false);
					}

					//check if possible to forward
					if ( (current_level_tx[o] == ack_tx[o].read()) &&
							(buffer_full_status_tx[o].read().mask[vc] == false) ) 
					{
						LOG << "Input[" << i << "][" << vc << "] forwarded to Output[" << o << "], flit: " << flit << endl;

						flit_tx[o].write(flit);
						current_level_tx[o] = 1 - current_level_tx[o];
						req_tx[o].write(current_level_tx[o]);
						buffer[i][vc].Pop();
						
						/* Power & Stats ------------------------------------------------- */
						if (o == DIRECTION_HUB) 
							power.r2hLink();
						else
							power.r2rLink();

						power.bufferRouterPop();
						power.crossBar();

						//if output direction is HUB direction, then E/O conversion needs to take place
						//this is Body flits sent to optical mesh. Add power for conversion
						if( o==DIRECTION_HUB)
							power.opticalInterface();

						if (o == DIRECTION_LOCAL) 
						{
							power.networkInterface();
							LOG << "Consumed flit " << flit << endl;
														
							stats.receivedFlit(sc_time_stamp().to_double() / GlobalParams::clock_period_ps, flit);
							if (GlobalParams:: max_volume_to_be_drained) 
							{
								if (drained_volume >= GlobalParams:: max_volume_to_be_drained)
									sc_stop();
								else 
								{
									drained_volume++;
									local_drained++;
								}
							}
						} 
						else if (i != DIRECTION_LOCAL) // not generated locally
							routed_flits++;
					}
					else
					{
						LOG << " Cannot forward Input[" << i << "][" << vc << "] to Output[" << o << "], flit: " << flit << endl;
						LOG << " **DEBUG buffer_full_status_tx " << buffer_full_status_tx[o].read().mask[vc] << endl;
					}
				}
			} // end of if(GlobalParams::use_optinoc)

			//all tx related already done for optical communication, remaining code to be skipped (current iteration)
			if(GlobalParams::use_optinoc)
				continue;

		  	if (flit.flit_type == FLIT_TYPE_HEAD) 
		    {
				// prepare data for routing
				RouteData route_data;
				route_data.current_id = local_id;
				//LOG<< "current_id= "<< route_data.current_id <<" for sending " << flit << endl;
				route_data.src_id = flit.src_id;
				route_data.dst_id = flit.dst_id;
				route_data.dir_in = i;
				route_data.vc_id = flit.vc_id;

				// TODO: see PER POSTERI (adaptive routing should not recompute route if already reserved)
				int o = route(route_data);

				// manage special case of target hub not directly connected to destination
				if (o>=DIRECTION_HUB_RELAY)
				{
					Flit f = buffer[i][vc].Pop();
					f.hub_relay_node = o-DIRECTION_HUB_RELAY;
					buffer[i][vc].Push(f);
					o = DIRECTION_HUB;
				}

		      	TReservation r;
		      	r.input = i;
		      	r.vc = vc;

		      	LOG << " checking availability of Output[" << o << "] for Input[" << i << "][" << vc << "] flit " << flit << endl;

		      	int rt_status = reservation_table.checkReservation(r,o);

		      	if (rt_status == RT_AVAILABLE) 
		    	{
					LOG << " reserving direction " << o << " for flit " << flit << endl;
					reservation_table.reserve(r, o);
		      	}
				else if (rt_status == RT_ALREADY_SAME)
				{
				LOG << " RT_ALREADY_SAME reserved direction " << o << " for flit " << flit << endl;
				}
				else if (rt_status == RT_OUTVC_BUSY)
				{
				LOG << " RT_OUTVC_BUSY reservation direction " << o << " for flit " << flit << endl;
				}
				else if (rt_status == RT_ALREADY_OTHER_OUT)
				{
				LOG  << "RT_ALREADY_OTHER_OUT: another output previously reserved for the same flit " << endl;
				}
				else assert(false); // no meaningful status here
		    }
		}
	  }
	    start_from_vc[i] = (start_from_vc[i]+1)%GlobalParams::n_virtual_channels;
	}

      start_from_port = (start_from_port + 1) % (DIRECTIONS + 2);

	//all optical related forwarding already doen above
	if(GlobalParams::use_optinoc)
		return;
	
      // 2nd phase: Forwarding
      //if (local_id==6) LOG<<"*TX*****local_id="<<local_id<<"__ack_tx[0]= "<<ack_tx[0].read()<<endl;
      for (int i = 0; i < DIRECTIONS + 2; i++) 
      { 
	  vector<pair<int,int> > reservations = reservation_table.getReservations(i);
	  
	  if (reservations.size()!=0)
	  {

	      int rnd_idx = rand()%reservations.size();

	      int o = reservations[rnd_idx].first;
	      int vc = reservations[rnd_idx].second;
	     // LOG<< "found reservation from input= " << i << "_to output= "<<o<<endl;
	      // can happen
	      if (!buffer[i][vc].IsEmpty())  
	      {
		  // power contribution already computed in 1st phase
		  Flit flit = buffer[i][vc].Front();
		  //LOG<< "*****TX***Direction= "<<i<< "************"<<endl;
		  //LOG<<"_cl_tx="<<current_level_tx[o]<<"req_tx="<<req_tx[o].read()<<" _ack= "<<ack_tx[o].read()<< endl;
		  
		  if ( (current_level_tx[o] == ack_tx[o].read()) &&
		       (buffer_full_status_tx[o].read().mask[vc] == false) ) 
		  {
		      //if (GlobalParams::verbose_mode > VERBOSE_OFF) 
		      LOG << "Input[" << i << "][" << vc << "] forwarded to Output[" << o << "], flit: " << flit << endl;

		      flit_tx[o].write(flit);
		      current_level_tx[o] = 1 - current_level_tx[o];
		      req_tx[o].write(current_level_tx[o]);
		      buffer[i][vc].Pop();
			  //logging
			//   if(i==DIRECTION_HUB)
			//   	cout<<local_id<<" POPPED"<<endl;

			//clear reservation if the transmitted flit is tail flit or one of the control flits
		      if (flit.flit_type == FLIT_TYPE_TAIL || flit.flit_type == FLIT_TYPE_REQ || flit.flit_type == FLIT_TYPE_ACK || flit.flit_type == FLIT_TYPE_DES)
		      {
			  	TReservation r;
			  	r.input = i;
			  	r.vc = vc;
			  	reservation_table.release(r,o);
		      }

		      /* Power & Stats ------------------------------------------------- */
		      if (o == DIRECTION_HUB) power.r2hLink();
		      else
			  power.r2rLink();

		      power.bufferRouterPop();
		      power.crossBar();

		      if (o == DIRECTION_LOCAL) 
		      {
			  power.networkInterface();
			  LOG << "Consumed flit " << flit << endl;
			  stats.receivedFlit(sc_time_stamp().to_double() / GlobalParams::clock_period_ps, flit);
			  if (GlobalParams:: max_volume_to_be_drained) 
			  {
			      if (drained_volume >= GlobalParams:: max_volume_to_be_drained)
				  sc_stop();
			      else 
			      {
				  drained_volume++;
				  local_drained++;
			      }
			  }
		      } 
		      else if (i != DIRECTION_LOCAL) // not generated locally
			  routed_flits++;
		      /* End Power & Stats ------------------------------------------------- */
			 //LOG<<"END_OK_cl_tx="<<current_level_tx[o]<<"_req_tx="<<req_tx[o].read()<<" _ack= "<<ack_tx[o].read()<< endl;
		  }
		  else
		  {
			  LOG << " Cannot forward Input[" << i << "][" << vc << "] to Output[" << o << "], flit: " << flit << endl;
		      //LOG << " **DEBUG APB: current_level_tx: " << current_level_tx[o] << " ack_tx: " << ack_tx[o].read() << endl;
		      LOG << " **DEBUG buffer_full_status_tx " << buffer_full_status_tx[o].read().mask[vc] << endl;

		  	//LOG<<"END_NO_cl_tx="<<current_level_tx[o]<<"_req_tx="<<req_tx[o].read()<<" _ack= "<<ack_tx[o].read()<< endl;
		      /*
		      if (flit.flit_type == FLIT_TYPE_HEAD)
			  reservation_table.release(i,flit.vc_id,o);
			  */
		  }
	      }
	  } // if not reserved 
	 // else LOG<<"we have no reservation for direction "<<i<< endl;
      } // for loop directions

      if ((int)(sc_time_stamp().to_double() / GlobalParams::clock_period_ps)%2==0)
	  reservation_table.updateIndex();
    }   
}

NoP_data Router::getCurrentNoPData()
{
    NoP_data NoP_data;

    for (int j = 0; j < DIRECTIONS; j++) {
	try {
		NoP_data.channel_status_neighbor[j].free_slots = free_slots_neighbor[j].read();
		NoP_data.channel_status_neighbor[j].available = (reservation_table.isNotReserved(j));
	}
	catch (int e)
	{
	    if (e!=NOT_VALID) assert(false);
	    // Nothing to do if an NOT_VALID direction is caught
	};
    }

    NoP_data.sender_id = local_id;

    return NoP_data;
}

void Router::perCycleUpdate()
{
    if (reset.read()) 
	{
		for (int i = 0; i < DIRECTIONS + 1; i++)
			free_slots[i].write(buffer[i][DEFAULT_VC].GetMaxBufferSize());
    } 
	else 
	{
        selectionStrategy->perCycleUpdate(this);

		power.leakageRouter();
		for (int i = 0; i < DIRECTIONS + 1; i++)
		{
			for (int vc=0;vc<GlobalParams::n_virtual_channels;vc++)
			{
			power.leakageBufferRouter();
			power.leakageLinkRouter2Router();
			}
		}
		//after each cycle, add the power consumed by MRs in on-state related to current router
		if(GlobalParams::use_optinoc)
			power.mrOnState(numOnStateMR);
		else
			power.leakageLinkRouter2Hub();
    }
}

vector<int> Router::nextDeltaHops(RouteData rd) {

	if (GlobalParams::topology == TOPOLOGY_MESH)
	{
		cout << "Mesh topologies are not supported for nextDeltaHops() ";
		assert(false);
	}
	// annotate the initial nodes
	int src = rd.src_id;
	int dst = rd.dst_id;

	int current_node = src;
	vector<int> direction; // initially is empty
	vector<int> next_hops;

	int sw = GlobalParams::n_delta_tiles/2; //sw: switch number in each stage
	int stg = log2(GlobalParams::n_delta_tiles);
	int c;
	//---From Source to stage 0 (return the sw attached to the source)---
	//Topology omega 
	if (GlobalParams::topology == TOPOLOGY_OMEGA) 	
	{
	if(current_node < (GlobalParams::n_delta_tiles/2))	
		 c = current_node;
	else if(current_node >= (GlobalParams::n_delta_tiles/2))	
		 c = (current_node - (GlobalParams::n_delta_tiles/2));		
	}
	//Other delta topologies: Butterfly and baseline
	else if ((GlobalParams::topology == TOPOLOGY_BUTTERFLY)||(GlobalParams::topology == TOPOLOGY_BASELINE))
	{
		 c =  (current_node >>1);
	}

		Coord temp_coord;
		temp_coord.x = 0;
		temp_coord.y = c;
		int N = coord2Id(temp_coord);

		next_hops.push_back(N);
		current_node = N;
	
	
   //---From stage 0 to Destination---
	int current_stage = 0;

	while (current_stage<stg-1)
	{
		Coord new_coord;
		int y = id2Coord(current_node).y;

		rd.current_id = current_node;
		direction = routingAlgorithm->route(this, rd);

		int bit_to_check = stg - current_stage - 1;

		int bit_checked = (y & (1 << (bit_to_check - 1)))>0 ? 1:0;

		// computes next node coords
		new_coord.x = current_stage + 1;
		if (bit_checked ^ direction[0])
			new_coord.y = toggleKthBit(y, bit_to_check);
		else
			new_coord.y = y;

		current_node = coord2Id(new_coord);
		next_hops.push_back(current_node);
		current_stage = id2Coord(current_node).x;
	}

	next_hops.push_back(dst);

	return next_hops;

}

//reaches this function if destination is different from local id
vector < int > Router::routingFunction(const RouteData & route_data)
{
	//if wireless mesh enabled
	if (GlobalParams::use_winoc)
	{
		// - If the current node C and the destination D are connected to an radiohub, use wireless
		// - If D is not directly connected to a radio hub, wireless
		// communication can still  be used if some intermediate node "I" in the routing
		// path is reachable from current node C.
		// - Since further wired hops will be required from I -> D, a threshold "winoc_dst_hops"
		// can be specified (via command line) to determine the max distance from the intermediate
		// node I and the destination D.
		// - NOTE: default threshold is 0, which means I=D, i.e., we explicitly ask the destination D to be connected to the
		// target radio hub
		if (hasRadioHub(local_id))
		{
			// Check if destination is directly connected to an hub
			if ( hasRadioHub(route_data.dst_id) &&
				 !sameRadioHub(local_id,route_data.dst_id) )
			{
                map<int, int>::iterator it1 = GlobalParams::hub_for_tile.find(route_data.dst_id);
                map<int, int>::iterator it2 = GlobalParams::hub_for_tile.find(route_data.current_id);

                if (connectedHubs(it1->second,it2->second))
                {
                    LOG << "Destination node " << route_data.dst_id << " is directly connected to a reachable RadioHub" << endl;
                    vector<int> dirv;
                    dirv.push_back(DIRECTION_HUB);
                    return dirv;
                }
			}
			// let's check whether some node in the route has an acceptable distance to the dst
            if (GlobalParams::winoc_dst_hops>0)
            {
                // TODO: for the moment, just print the set of nexts hops to check everything is ok
                LOG << "NEXT_DELTA_HOPS (from node " << route_data.src_id << " to " << route_data.dst_id << ") >>>> :";
                vector<int> nexthops;
                nexthops = nextDeltaHops(route_data);
                //for (int i=0;i<nexthops.size();i++) cout << "(" << nexthops[i] <<")-->";
                //cout << endl;
                for (int i=1;i<=GlobalParams::winoc_dst_hops;i++)
				{
                	int dest_position = nexthops.size()-1;
                	int candidate_hop = nexthops[dest_position-i];
					if ( hasRadioHub(candidate_hop) && !sameRadioHub(local_id,candidate_hop) ) {
						//LOG << "Checking candidate hop " << candidate_hop << " ... It's OK!" << endl;
						LOG << "Relaying to hub-connected node " << candidate_hop << " to reach destination " << route_data.dst_id << endl;
						vector<int> dirv;
						dirv.push_back(DIRECTION_HUB_RELAY+candidate_hop);
						return dirv;
					}
					//else
					// LOG << "Checking candidate hop " << candidate_hop << " ... NOT OK" << endl;
				}
            }
		}
	}
	//if optical mesh enabled, always forward flit to optical mesh
	if(GlobalParams::use_optinoc)
	{
		vector<int> dirv;
		dirv.push_back(DIRECTION_HUB);
		return dirv;
	}
	// TODO: fix all the deprecated verbose mode logs
	if (GlobalParams::verbose_mode > VERBOSE_OFF)
		LOG << "Wired routing for dst = " << route_data.dst_id << endl;

	// not wireless direction taken, apply normal routing
	return routingAlgorithm->route(this, route_data);
}

int Router::route(const RouteData & route_data)
{

    if (route_data.dst_id == local_id)
	return DIRECTION_LOCAL;

    power.routing();
    vector < int >candidate_channels = routingFunction(route_data);

    power.selection();
    return selectionFunction(candidate_channels, route_data);
}

void Router::NoP_report() const
{
    NoP_data NoP_tmp;
	LOG << "NoP report: " << endl;

    for (int i = 0; i < DIRECTIONS; i++) {
	NoP_tmp = NoP_data_in[i].read();
	if (NoP_tmp.sender_id != NOT_VALID)
	    cout << NoP_tmp;
    }
}

//---------------------------------------------------------------------------

int Router::NoPScore(const NoP_data & nop_data,
			  const vector < int >&nop_channels) const
{
    int score = 0;

    for (unsigned int i = 0; i < nop_channels.size(); i++) {
	int available;

	if (nop_data.channel_status_neighbor[nop_channels[i]].available)
	    available = 1;
	else
	    available = 0;

	int free_slots =
	    nop_data.channel_status_neighbor[nop_channels[i]].free_slots;

	score += available * free_slots;
    }

    return score;
}

int Router::selectionFunction(const vector < int >&directions,
				   const RouteData & route_data)
{
    // not so elegant but fast escape ;)
    if (directions.size() == 1)
	return directions[0];

    return selectionStrategy->apply(this, directions, route_data);
}

void Router::configure(const int _id,
			    const double _warm_up_time,
			    const unsigned int _max_buffer_size,
			    GlobalRoutingTable & grt)
{
    local_id = _id;
    stats.configure(_id, _warm_up_time);

    start_from_port = DIRECTION_LOCAL;
  

    if (grt.isValid())
	routing_table.configure(grt, _id);

    reservation_table.setSize(DIRECTIONS+2);

    for (int i = 0; i < DIRECTIONS + 2; i++)
    {
	for (int vc = 0; vc < GlobalParams::n_virtual_channels; vc++)
	{
	    buffer[i][vc].SetMaxBufferSize(_max_buffer_size);
	    buffer[i][vc].setLabel(string(name())+"->buffer["+i_to_string(i)+"]");
	}
	start_from_vc[i] = 0;
    }

	//NEW, initialise the newly created buffer_ack
	for (int vc = 0; vc < GlobalParams::n_virtual_channels; vc++)
    {
        buffer_ack[vc].SetMaxBufferSize(_max_buffer_size);
        buffer_ack[vc].setLabel(string(name())+"->buffer_ack");
    }

    if (GlobalParams::topology == TOPOLOGY_MESH)
    {
	int row = _id / GlobalParams::mesh_dim_x;
	int col = _id % GlobalParams::mesh_dim_x;

	for (int vc = 0; vc<GlobalParams::n_virtual_channels; vc++)
	{
	    if (row == 0)
	      buffer[DIRECTION_NORTH][vc].Disable();
	    if (row == GlobalParams::mesh_dim_y-1)
	      buffer[DIRECTION_SOUTH][vc].Disable();
	    if (col == 0)
	      buffer[DIRECTION_WEST][vc].Disable();
	    if (col == GlobalParams::mesh_dim_x-1)
	      buffer[DIRECTION_EAST][vc].Disable();
	}
    }

}

unsigned long Router::getRoutedFlits()
{
    return routed_flits;
}


int Router::reflexDirection(int direction) const
{
    if (direction == DIRECTION_NORTH)
	return DIRECTION_SOUTH;
    if (direction == DIRECTION_EAST)
	return DIRECTION_WEST;
    if (direction == DIRECTION_WEST)
	return DIRECTION_EAST;
    if (direction == DIRECTION_SOUTH)
	return DIRECTION_NORTH;

    // you shouldn't be here
    assert(false);
    return NOT_VALID;
}

int Router::getNeighborId(int _id, int direction) const
{
    assert(GlobalParams::topology == TOPOLOGY_MESH);

    Coord my_coord = id2Coord(_id); 

    switch (direction) {
    case DIRECTION_NORTH:
	if (my_coord.y == 0)
	    return NOT_VALID;
	my_coord.y--;
	break;
    case DIRECTION_SOUTH:
	if (my_coord.y == GlobalParams::mesh_dim_y - 1)
	    return NOT_VALID;
	my_coord.y++;
	break;
    case DIRECTION_EAST:
	if (my_coord.x == GlobalParams::mesh_dim_x - 1)
	    return NOT_VALID;
	my_coord.x++;
	break;
    case DIRECTION_WEST:
	if (my_coord.x == 0)
	    return NOT_VALID;
	my_coord.x--;
	break;
    default:
	LOG << "Direction not valid : " << direction;
	assert(false);
    }

    int neighbor_id = coord2Id(my_coord);

    return neighbor_id;
}

bool Router::inCongestion()
{
    for (int i = 0; i < DIRECTIONS; i++) {

	if (free_slots_neighbor[i]==NOT_VALID) continue;

	int flits = GlobalParams::buffer_depth - free_slots_neighbor[i];
	if (flits > (int) (GlobalParams::buffer_depth * GlobalParams::dyad_threshold))
	    return true;
    }

    return false;
}

void Router::ShowBuffersStats(std::ostream & out)
{
  for (int i=0; i<DIRECTIONS+2; i++)
      for (int vc=0; vc<GlobalParams::n_virtual_channels;vc++)
	    buffer[i][vc].ShowStats(out);
}


bool Router::connectedHubs(int src_hub, int dst_hub) {
    vector<int> &first = GlobalParams::hub_configuration[src_hub].txChannels;
    vector<int> &second = GlobalParams::hub_configuration[dst_hub].rxChannels;

    vector<int> intersection;

    for (unsigned int i = 0; i < first.size(); i++) {
        for (unsigned int j = 0; j < second.size(); j++) {
            if (first[i] == second[j])
                intersection.push_back(first[i]);
        }
    }

    if (intersection.size() == 0)
        return false;
    else
        return true;
}
