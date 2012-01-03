/*
 * @author Ugo Colesanti
 * @author Silvia Santini
 * @version 1.0 (January 3, 2012)
 */

/*
 * Copyright (c) 2012 Sapienza University of Rome.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the Sapienza University of Rome nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL SAPIENZA
 * UNIVERSITY OF ROME OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Copyright (c) 2012 ETH Zurich.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of ETH Zurich nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL ETH
 * ZURICH OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "StatsCollector.h"
#include "VirtualMobilityManager.h"
#include "CtpRoutingEngine.h"

#define index(x,y,c) (y*c+x)

Define_Module(StatsCollector);


void StatsCollector::initialize(){

	totNodes = getParentModule()->par("numNodes");
	rxBeacons = new int[totNodes*totNodes];
	txBeacons = new int[totNodes] ;
	cfeStats = new int[totNodes*22];

	appRx = 0 ;
	appTx = 0 ;
	appDupRx = 0 ;

for(int i=0;i<totNodes;i++){
	txBeacons[i]=0;
}
for(int i=0; i<totNodes*totNodes; i++){
	rxBeacons[i]=0;
}
for(int i=0; i<totNodes*22; i++){
	cfeStats[i]=0;
}

	simulation.getSystemModule()->subscribe("BeaconRx",this) ;
	simulation.getSystemModule()->subscribe("BeaconTx",this) ;

	simulation.getSystemModule()->subscribe("AppTx",this) ;
	simulation.getSystemModule()->subscribe("AppRx",this) ;
	simulation.getSystemModule()->subscribe("AppDuplicateRx",this) ;

	simulation.getSystemModule()->subscribe("CfeTxDroppedNotRunning",this) ;
	simulation.getSystemModule()->subscribe("CfeTxDroppedTooLong",this) ;
	simulation.getSystemModule()->subscribe("CfeTxDroppedEbusy",this) ;
	simulation.getSystemModule()->subscribe("CfeTxDelayedNoRoute",this) ;
	simulation.getSystemModule()->subscribe("CfeTxDelayedParentCongested",this) ;
	simulation.getSystemModule()->subscribe("CfeDupDroppedInSentCache",this) ;
	simulation.getSystemModule()->subscribe("CfeTxOkIsLoopback",this) ;
	simulation.getSystemModule()->subscribe("CfeTxOkClient",this) ;
	simulation.getSystemModule()->subscribe("CfeTxOkForwarded",this) ;
	simulation.getSystemModule()->subscribe("CfeTxDroppedRadioOff",this) ;
	simulation.getSystemModule()->subscribe("CfeTxDroppedRadioEbusy",this) ;
	simulation.getSystemModule()->subscribe("CfeTxDoneFailed",this) ;
	simulation.getSystemModule()->subscribe("CfeTxDoneNoAck",this) ;
	simulation.getSystemModule()->subscribe("CfeTxDoneDroppedMaxRetriesClient",this) ;
	simulation.getSystemModule()->subscribe("CfeTxDoneDroppedMacRetriesForwarded",this) ;
	simulation.getSystemModule()->subscribe("CfeTxDoneOkClient",this) ;
	simulation.getSystemModule()->subscribe("CfeTxDoneOkForwarded",this) ;
	simulation.getSystemModule()->subscribe("CfeTxDroppedMessagePoolFull",this) ;
	simulation.getSystemModule()->subscribe("CfeTxDroppedQentryPoolFull",this) ;
	simulation.getSystemModule()->subscribe("CfeTxDroppedQentryFull",this) ;
	simulation.getSystemModule()->subscribe("CfeTxDroppedMessagePoolFull2",this) ;
	simulation.getSystemModule()->subscribe("CfeTxDroppedSendQueueFull",this) ;



}

void StatsCollector::receiveSignal(cComponent *src, simsignal_t id, long l){
	if(registerSignal("BeaconTx") == id){
		int idx = (int) l ;
		txBeacons[idx]++ ;
	}
	else if(registerSignal("AppTx") == id){
		appTx++ ;
	}
	else if(registerSignal("AppRx") == id){
		appRx++ ;
	}
	else if(registerSignal("AppDuplicateRx") == id){
		appDupRx++ ;
	}
	else if(registerSignal("CfeTxDroppedNotRunning") == id){
		cfeStats[index((int)l,CfeTxDroppedNotRunning,totNodes)]++ ;
	}
	else if(registerSignal("CfeTxDroppedTooLong") == id){
			cfeStats[index((int)l,CfeTxDroppedTooLong,totNodes)]++ ;
	}
	else if(registerSignal("CfeTxDroppedEbusy") == id){
				cfeStats[index((int)l,CfeTxDroppedEbusy,totNodes)]++ ;
		}
	else if(registerSignal("CfeTxDelayedNoRoute") == id){
				cfeStats[index((int)l,CfeTxDelayedNoRoute,totNodes)]++ ;
		}
	else if(registerSignal("CfeTxDelayedParentCongested") == id){
				cfeStats[index((int)l,CfeTxDelayedParentCongested,totNodes)]++ ;
		}
	else if(registerSignal("CfeDupDroppedInSentCache") == id){
				cfeStats[index((int)l,CfeDupDroppedInSentCache,totNodes)]++ ;
		}
	else if(registerSignal("CfeTxOkIsLoopback") == id){
				cfeStats[index((int)l,CfeTxOkIsLoopback,totNodes)]++ ;
		}
	else if(registerSignal("CfeTxOkClient") == id){
				cfeStats[index((int)l,CfeTxOkClient,totNodes)]++ ;
		}
	else if(registerSignal("CfeTxOkForwarded") == id){
				cfeStats[index((int)l,CfeTxOkForwarded,totNodes)]++ ;
		}
	else if(registerSignal("CfeTxDroppedRadioOff") == id){
				cfeStats[index((int)l,CfeTxDroppedRadioOff,totNodes)]++ ;
		}
	else if(registerSignal("CfeTxDroppedRadioEbusy") == id){
				cfeStats[index((int)l,CfeTxDroppedRadioEbusy,totNodes)]++ ;
		}
	else if(registerSignal("CfeTxDoneFailed") == id){
				cfeStats[index((int)l,CfeTxDoneFailed,totNodes)]++ ;
		}
	else if(registerSignal("CfeTxDoneNoAck") == id){
				cfeStats[index((int)l,CfeTxDoneNoAck,totNodes)]++ ;
		}
	else if(registerSignal("CfeTxDoneDroppedMaxRetriesClient") == id){
				cfeStats[index((int)l,CfeTxDoneDroppedMaxRetriesClient,totNodes)]++ ;
		}
	else if(registerSignal("CfeTxDoneDroppedMacRetriesForwarded") == id){
				cfeStats[index((int)l,CfeTxDoneDroppedMacRetriesForwarded,totNodes)]++ ;
		}
	else if(registerSignal("CfeTxDoneOkClient") == id){
				cfeStats[index((int)l,CfeTxDoneOkClient,totNodes)]++ ;
		}
	else if(registerSignal("CfeTxDoneOkForwarded") == id){
				cfeStats[index((int)l,CfeTxDoneOkForwarded,totNodes)]++ ;
		}
	else if(registerSignal("CfeTxDroppedMessagePoolFull") == id){
				cfeStats[index((int)l,CfeTxDroppedMessagePoolFull,totNodes)]++ ;
		}
	else if(registerSignal("CfeTxDroppedQentryPoolFull") == id){
				cfeStats[index((int)l,CfeTxDroppedQentryPoolFull,totNodes)]++ ;
		}
	else if(registerSignal("CfeTxDroppedQentryFull") == id){
				cfeStats[index((int)l,CfeTxDroppedQentryFull,totNodes)]++ ;
		}
	else if(registerSignal("CfeTxDroppedMessagePoolFull2") == id){
				cfeStats[index((int)l,CfeTxDroppedMessagePoolFull2,totNodes)]++ ;
		}
	else if(registerSignal("CfeTxDroppedSendQueueFull") == id){
				cfeStats[index((int)l,CfeTxDroppedSendQueueFull,totNodes)]++ ;
		}
	else opp_error("Unknown signal received. 2") ;
}

void StatsCollector::receiveSignal(cComponent *src, simsignal_t id, cObject* obj){
	
	if(registerSignal("BeaconRx") == id){
		beaconRx* brx = check_and_cast<beaconRx*>(obj) ;
		rxBeacons[index(brx->id,brx->from,totNodes)]++ ;
	}
	else opp_error("Unknown signal.") ;
}

void StatsCollector::finishSpecific(){
	string runNumber = (ev.getConfig()->getConfigValue("seed-set")) ;

	ostringstream outputTopologyName ;
	outputTopologyName<<"topology-"<<runNumber<<".txt" ;
	ostringstream outputTopologyStream;
	for(int i = 0 ; i < totNodes ; i++){
		VirtualMobilityManager* mm = check_and_cast<VirtualMobilityManager*>(simulation.getSystemModule()->getSubmodule("node",i)->getSubmodule("MobilityManager")) ;
		outputTopologyStream<<i<<" "<<mm->getLocation().x<<" "<<mm->getLocation().y<<"\n"  ;
	}
	printFile(outputTopologyStream.str(),outputTopologyName.str()) ;

	// Print metrics
	ostringstream outputRxName ;
	outputRxName<<"rxBeacons-"<<runNumber<<".txt" ;
	ostringstream outputRxStream;

	for(int i = 0; i<totNodes; i++){
		for(int j=0; j<totNodes; j++){
			outputRxStream<<rxBeacons[index(i,j,totNodes)]<<" ";		
		}
		outputRxStream<<"\n" ;
	}
	printFile(outputRxStream.str(),outputRxName.str()) ;
	
	
	ostringstream outputTxName ;
	outputTxName<<"txBeacons-"<<runNumber<<".txt" ;
	ostringstream outputTxStream;

	for(int i = 0; i<totNodes; i++){
		outputTxStream<<txBeacons[i]<<"\n";		
	}
	printFile(outputTxStream.str(),outputTxName.str()) ;
	

	ostringstream outputStatsName ;
	outputStatsName<<"stats-"<<runNumber<<".txt" ;
	ostringstream outputStatsStream;

	outputStatsStream<<"AppTx: "<<appTx<<"\n" ;
	outputStatsStream<<"AppRx: "<<appRx<<"\n";
	outputStatsStream<<"AppDupRx: "<<appDupRx<<"\n";

	printFile(outputStatsStream.str(),outputStatsName.str()) ;

	ostringstream outputCfeStatsName ;
	outputCfeStatsName<<"cfeStats-"<<runNumber<<".txt" ;
	ostringstream outputCfeStatsStream;

	for(int i = 0; i<totNodes; i++){
		for(int j=0; j<22; j++){
			outputCfeStatsStream<<cfeStats[index(i,j,totNodes)]<<" ";
		}
		outputCfeStatsStream<<"\n" ;
	}
	printFile(outputCfeStatsStream.str(),outputCfeStatsName.str()) ;


	simulation.getSystemModule()->unsubscribe("BeaconRx",this) ;
	simulation.getSystemModule()->unsubscribe("BeaconTx",this) ;


	simulation.getSystemModule()->unsubscribe("AppTx",this) ;
	simulation.getSystemModule()->unsubscribe("AppRx",this) ;
	simulation.getSystemModule()->unsubscribe("AppDuplicateRx",this) ;

	simulation.getSystemModule()->unsubscribe("CfeTxDroppedNotRunning",this) ;
	simulation.getSystemModule()->unsubscribe("CfeTxDroppedTooLong",this) ;
	simulation.getSystemModule()->unsubscribe("CfeTxDroppedEbusy",this) ;
	simulation.getSystemModule()->unsubscribe("CfeTxDelayedNoRoute",this) ;
	simulation.getSystemModule()->unsubscribe("CfeTxDelayedParentCongested",this) ;
	simulation.getSystemModule()->unsubscribe("CfeDupDroppedInSentCache",this) ;
	simulation.getSystemModule()->unsubscribe("CfeTxOkIsLoopback",this) ;
	simulation.getSystemModule()->unsubscribe("CfeTxOkClient",this) ;
	simulation.getSystemModule()->unsubscribe("CfeTxOkForwarded",this) ;
	simulation.getSystemModule()->unsubscribe("CfeTxDroppedRadioOff",this) ;
	simulation.getSystemModule()->unsubscribe("CfeTxDroppedRadioEbusy",this) ;
	simulation.getSystemModule()->unsubscribe("CfeTxDoneFailed",this) ;
	simulation.getSystemModule()->unsubscribe("CfeTxDoneNoAck",this) ;
	simulation.getSystemModule()->unsubscribe("CfeTxDoneDroppedMaxRetriesClient",this) ;
	simulation.getSystemModule()->unsubscribe("CfeTxDoneDroppedMacRetriesForwarded",this) ;
	simulation.getSystemModule()->unsubscribe("CfeTxDoneOkClient",this) ;
	simulation.getSystemModule()->unsubscribe("CfeTxDoneOkForwarded",this) ;
	simulation.getSystemModule()->unsubscribe("CfeTxDroppedMessagePoolFull",this) ;
	simulation.getSystemModule()->unsubscribe("CfeTxDroppedQentryPoolFull",this) ;
	simulation.getSystemModule()->unsubscribe("CfeTxDroppedQentryFull",this) ;
	simulation.getSystemModule()->unsubscribe("CfeTxDroppedMessagePoolFull2",this) ;
	simulation.getSystemModule()->unsubscribe("CfeTxDroppedSendQueueFull",this) ;

}

void StatsCollector::printFile(const string& outputFile, const string& filename){
		ofstream myOfile ;
		myOfile.open(filename.c_str());
		myOfile<<outputFile ;
		myOfile.close() ;
}

