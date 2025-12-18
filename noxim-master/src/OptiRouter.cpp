/*
 * Noxim - the NoC Simulator
 *
 * (C) 2005-2018 by the University of Catania
 * For the complete list of authors refer to file ../doc/AUTHORS.txt
 * For the license applied to these sources refer to file ../doc/LICENSE.txt
 *
 * This file contains the implementation of the router
 */

#include "OptiRouter.h"

void OptiRouter::process()
{
    txProcess();
    rxProcess();
}

//receiving flits process of optical router
void OptiRouter::rxProcess()
{
    if (reset.read()) 
    {
        TBufferFullStatus bfs;
        // Clear outputs and indexes of receiving protocol
        ack_electrical_rx.write(0);
        current_level_rx = 0;
        buffer_full_status_rx.write(bfs);
        routed_flits = 0;
        local_drained = 0;
        //NEW
        for (int i = 0; i < DIRECTIONS; i++)
        {
            last_received_flit = flit_optical_rx[i].read();
        }
        // last_received_flit = flit_optical_rx.read();
    } 
    else 
    { 
        // To accept a new flit, the following conditions must match:
        // 1) there is an incoming request
        // 2) there is a free slot in the input buffer

        //Reading signal from electrical mesh

        //new unread signal being transmitted
        if (req_electrical_rx.read() == 1 - current_level_rx)
        { 
            Flit received_flit = flit_electrical_rx.read();
            int vc = received_flit.vc_id;

            //there is free slot in buffer
            if (!buffer[vc].IsFull()) 
            {
                // Store the incoming flit in the circular buffer
                buffer[vc].Push(received_flit);
                LOG << " Flit " << received_flit << " collected from Input[Electrical layer] of OPTI" << endl;

                power.bufferRouterPush();

                // Negate the old value for Alternating Bit Protocol (ABP)
                current_level_rx = 1 - current_level_rx;
            }
            else  // buffer full
            {
                // should not happen with the new TBufferFullStatus control signals    
                LOG << " Flit " << received_flit << " buffer full Input Electrical layer of OPTI" << endl;
                //cout<<"Buffer full from electrical mesh read"<<endl;
                assert(false);
            }
        }
        //send back acknowledgement denoting current level
        ack_electrical_rx.write(current_level_rx);
        
        // updates the mask of VCs to prevent incoming data on full buffers
        TBufferFullStatus bfs;
        for (int vc=0;vc<GlobalParams::n_virtual_channels;vc++)
            bfs.mask[vc] = buffer[vc].IsFull();
        
        buffer_full_status_rx.write(bfs);

        //Read signal from optical mesh
        //returns true if the signal value changed

        //NEW

        for(int i = 0; i < DIRECTIONS; i++)
        {
            if(!(last_received_flit==flit_optical_rx[i].read()))  
            {
                //keep reading the incoming signal each cycle until tail flit is encountered
                Flit received_flit = flit_optical_rx[i].read();
                last_received_flit = received_flit;
                int vc = received_flit.vc_id;
                
                //there is free slot in buffer (always there will be)
                if (!buffer_opti[vc].IsFull()) 
                {
                    buffer_opti[vc].Push(received_flit);
                    LOG << " Flit " << received_flit << " collected from Optical layer of Opti:" <<local_id << endl;

                    power.bufferRouterPush();
                }
                else  // buffer full (shud not happen at all)
                {   
                    LOG << " Flit " << received_flit << " buffer full from Optical layer of Opti:" <<local_id << endl;
                    //cout<<"Buffer full from Optical mesh read"<<endl;
                    //assert(false);
                }

                
            }
        }

        // if(!(last_received_flit==flit_optical_rx.read()))  
        // {
        //     //keep reading the incoming signal each cycle until tail flit is encountered
        //     Flit received_flit = flit_optical_rx.read();
        //     last_received_flit = received_flit;
        //     int vc = received_flit.vc_id;
            
        //     //there is free slot in buffer (always there will be)
        //     if (!buffer_opti[vc].IsFull()) 
        //     {
        //         buffer_opti[vc].Push(received_flit);
        //         LOG << " Flit " << received_flit << " collected from Optical layer of Opti:" <<local_id << endl;

        //         power.bufferRouterPush();
        //     }
        //     else  // buffer full (shud not happen at all)
        //     {   
        //         LOG << " Flit " << received_flit << " buffer full from Optical layer of Opti:" <<local_id << endl;
        //         //cout<<"Buffer full from Optical mesh read"<<endl;
        //         //assert(false);
        //     }

            
        // }
    }
}

//transmitting flit process of optical router
void OptiRouter::txProcess()
{
    if (reset.read()) 
    {
        // Clear outputs and indexes of transmitting protocol
        req_electrical_tx.write(0);
        current_level_tx = 0;
    } 
    else 
    { 
        //transmit flits to electrical mesh first (from optical mesh)
        
        int vc=0;
        //check if there anything in buffer_opti
        if (!buffer_opti[vc].IsEmpty())  
        {
            Flit flit = buffer_opti[vc].Front();
            power.bufferRouterFront();
        
            //check if router has completed reading prev flit and ready to recieve new flit
            //check if router has space in its buffer to recieve new flit
            if ( (current_level_tx == ack_electrical_tx.read()) &&
                (buffer_full_status_tx.read().mask[vc] == false) ) 
            {
                //cout<<"Reaching transmit from buffer_opti to electrical"<<endl;
                LOG << "Flit forwarded from optical router to electrical[" << local_id << "] flit: " << flit << endl;

                //write current flit, change current_level and send request for transmitting
                //with updated current level to indicate new flit available to reciever
                flit_electrical_tx.write(flit);
                current_level_tx = 1 - current_level_tx;
                req_electrical_tx.write(current_level_tx);
                buffer_opti[vc].Pop();

                //POWER CALCULATIONS
                power.bufferRouterPop();
                power.crossBar();

                routed_flits++;
            }
            //transmitting was not possible, reciever hasn't read prev flit or no space in buffer
            else
            {
                //if the buffer got full for some reason
                if(buffer_full_status_tx.read().mask[vc])
                    LOG << "ISSUE!!!!" <<endl;
                //cout<<"cannot forward from optical to electrical:"<<buffer_full_status_tx.read().mask[vc]<<endl;
                LOG << " Cannot forward flit from optical to electrical[" << local_id << "], flit: " << flit << endl;
                //LOG << " **DEBUG APB: current_level_tx: " << current_level_tx[o] << " ack_tx: " << ack_tx[o].read() << endl;
                //LOG << " **DEBUG buffer_full_status_tx " << buffer_full_status_tx[o].read().mask[vc] << endl;
            }
        }

        //transmit flit from electrical mesh to destination optical router
        
        //check if there anything in buffer storing flits from electrical mesh
        if (!buffer[vc].IsEmpty())  
        {
            Flit flit = buffer[vc].Front();
            power.bufferRouterFront();

            int dst_id = flit.dst_id; 

            //transmit current flit to destination optical router
            LOG << "Flit transmitted from optical[" << local_id << "] to optical[" << dst_id << "] flit: " << flit << endl;
            // //NEW
            if(dst_id%4 ==0){
            flit_optical_tx[0][dst_id].write(flit);
            }
            else if(dst_id%4 ==1){
            flit_optical_tx[1][dst_id].write(flit);
            }
            else if(dst_id%4 ==2){
            flit_optical_tx[2][dst_id].write(flit);
            }
            else if(dst_id%4 ==3){
            flit_optical_tx[3][dst_id].write(flit);
            }
            
            buffer[vc].Pop();

            //POWER CALCULATIONS
            power.bufferRouterPop();
            routed_flits++;
            
        }
    }   
}

//update power consumption for idle activities such as leakage
void OptiRouter::perCycleUpdate()
{
    if (reset.read()) 
    {
    } 
    else 
    {
        power.leakageRouter();
        power.leakageBufferRouter();
        power.leakageLinkRouter2Router();
    }
}


//initialise optical router
void OptiRouter::configure(const int _id,
			    const double _warm_up_time,
			    const unsigned int _max_buffer_size)
{
    local_id = _id;
    stats.configure(_id, _warm_up_time);

    //initialise the buffer sizes for buffer holding flits from electrical mesh
    for (int vc = 0; vc < GlobalParams::n_virtual_channels; vc++)
    {
        buffer[vc].SetMaxBufferSize(_max_buffer_size);
        buffer[vc].setLabel(string(name())+"->buffer");
    }

    //initialise the buffer sizes for buffer holding flits from optical mesh
    for (int vc = 0; vc < GlobalParams::n_virtual_channels; vc++)
    {
        buffer_opti[vc].SetMaxBufferSize(_max_buffer_size);
        buffer_opti[vc].setLabel(string(name())+"->buffer_opti");
    }
}

unsigned long OptiRouter::getRoutedFlits()
{
    return routed_flits;
}

void OptiRouter::ShowBuffersStats(std::ostream & out)
{
    for (int vc=0; vc<GlobalParams::n_virtual_channels;vc++)
        buffer[vc].ShowStats(out);
}

