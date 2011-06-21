/****************************************************************************
 *  Copyright: National ICT Australia,  2007 - 2010                         *
 *  Developed at the ATP lab, Networked Systems research theme              *
 *  Author(s): Athanassios Boulis, Yuriy Tselishchev                        *
 *  This file is distributed under the terms in the attached LICENSE file.  *
 *  If you do not find this file, copies can be found by writing to:        *
 *                                                                          *
 *      NICTA, Locked Bag 9013, Alexandria, NSW 1435, Australia             *
 *      Attention:  License Inquiry.                                        *
 *                                                                          *  
 ****************************************************************************/

/*
 * @author Ugo Colesanti
 * @author Silvia Santini
 * @version 1.0 (January 27, 2011)
 */

#include "CtpTestingApplication.h"

Define_Module(CtpTestingApplication);

void CtpTestingApplication::startup()
{
	trace() <<"App started" ;
	packet_rate = par("packet_rate");
	recipientAddress = par("nextRecipient").stringValue();
	startupDelay = par("startupDelay");

	packet_spacing = packet_rate > 0 ? 1 / float (packet_rate) : -1;
	packet_info_table.clear();
	packet_ddr_table.clear() ;
	total_packets_received = 0;
	packets_lost_at_mac = 0;
	packets_lost_at_network = 0;
	dataSN = 0;

	if (packet_spacing > 0 && ( (recipientAddress.compare(SELF_NETWORK_ADDRESS) != 0) || (recipientAddress.compare(SELF_NETWORK_ADDRESS) != 99) || (recipientAddress.compare(SELF_NETWORK_ADDRESS) != 50) ))
		setTimer(SEND_PACKET, packet_spacing + startupDelay);
	else
		trace() << "Not sending packets";
}

void CtpTestingApplication::fromNetworkLayer(ApplicationGenericDataPacket * rcvPacket,
		const char *source, double rssi, double lqi)
{
	int sequenceNumber = rcvPacket->getSequenceNumber();
//	trace() << "Received packet from node " << source << ", SN = " << sequenceNumber;
//	if (recipientAddress.compare(SELF_NETWORK_ADDRESS) == 0 || recipientAddress.compare(SELF_NETWORK_ADDRESS) == 99) {
		trace() << "Received packet from node " << source << ", SN = " << sequenceNumber;
		//update_packets_received(atoi(source), sequenceNumber);
		update_ddr_per_sn(atoi(source),sequenceNumber) ;
//	} else {
//		toNetworkLayer(rcvPacket->dup(), recipientAddress.c_str());
//	}
}

void CtpTestingApplication::timerFiredCallback(int index)
{
	switch (index) {
		case SEND_PACKET:{
			if(dataSN<256){
			trace() << "Sending packet with SN " << dataSN;
			toNetworkLayer(createGenericDataPacket(0, dataSN), par("nextRecipient"));
			dataSN++;
			setTimer(SEND_PACKET, packet_spacing);
			}
			break;
		}
	}
}

// This method updates the number of packets received by node 0 from other nodes
void CtpTestingApplication::update_packets_received(int srcID, int SN)
{
	map < int, packet_info >::iterator i = packet_info_table.find(srcID);
	if (i == packet_info_table.end())
		declareOutput("Packets received", srcID);
	packet_info_table[srcID].packets_received[SN]++;
	if (packet_info_table[srcID].packets_received[SN] == 1)
		collectOutput("Packets received", srcID);
}

void CtpTestingApplication::update_ddr_per_sn(int srcID , int SN){
	map<int,int>::iterator i = packet_ddr_table.find(SN) ;
	if(i==packet_ddr_table.end()){
		trace()<<"New entry: "<<SN ;
		declareOutput("My Stats",SN) ;
	}
	if(packet_info_table[srcID].packets_received[SN]==1){ //duplicate reception
		collectOutput("My Stats",SN,"Duplicates") ;
	}
	else {
		packet_info_table[srcID].packets_received[SN]++;
		packet_ddr_table[SN] += 1 ;
		collectOutput("My Stats",SN,"DDR") ;
	}
	trace()<<"Received for SN "<<SN<<": "<<packet_ddr_table[SN] ;
}


// This method processes a received carrier sense interupt. Used only for demo purposes
// in some simulation. Feel free to comment out the trace command.  
void CtpTestingApplication::handleRadioControlMessage(RadioControlMessage *radioMsg)
{
	switch (radioMsg->getRadioControlMessageKind()) {
		case CARRIER_SENSE_INTERRUPT:
			trace() << "CS Interrupt received! current RSSI value is: " << radioModule->readRSSI();
                        break;
	}
}

