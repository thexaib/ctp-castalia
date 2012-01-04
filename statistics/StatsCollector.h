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

#include "CastaliaModule.h"

using namespace std;

enum{
	CfeTxDroppedNotRunning = 0 ,
	CfeTxDroppedTooLong = 1 ,
	CfeTxDroppedEbusy = 2 ,
	CfeTxDelayedNoRoute = 3 ,
	CfeTxDelayedParentCongested = 4 ,
	CfeDupDroppedInSentCache = 5 ,
	CfeTxOkIsLoopback = 6 ,
	CfeTxOkClient = 7 ,
	CfeTxOkForwarded = 8 ,
	CfeTxDroppedRadioOff = 9 ,
	CfeTxDroppedRadioEbusy = 10 ,
	CfeTxDoneFailed = 11 ,
	CfeTxDoneNoAck = 12 ,
	CfeTxDoneDroppedMaxRetriesClient = 13 ,
	CfeTxDoneDroppedMacRetriesForwarded = 14 ,
	CfeTxDoneOkClient = 15 ,
	CfeTxDoneOkForwarded = 16 ,
	CfeTxDroppedMessagePoolFull = 17 ,
	CfeTxDroppedQentryPoolFull = 18 ,
	CfeTxDroppedQentryFull = 19 ,
	CfeTxDroppedMessagePoolFull2 = 20 ,
	CfeTxDroppedSendQueueFull = 21 ,
};

struct metricMap{
	double falsePositive ;
	double sensing ;
	double received ;
	double duplicated ;
	double dropped_max_retries ;
};

struct phySample{
	double value ;
	double posX ;
	double posY ;
};


typedef map <long, metricMap> MetricMapByRound;

typedef map <long, phySample> PhySampleByNode ;

typedef map <long, PhySampleByNode> PhySampleByRound ;

typedef map <long,int> ActProbByNode ;

typedef map <long, ActProbByNode> ActProbByRound ;

class StatsCollector: public CastaliaModule,cListener{

public:
	virtual void receiveSignal(cComponent *src, simsignal_t id, long l) ;
	virtual void receiveSignal(cComponent *src, simsignal_t id, cObject* obj) ;

protected:
	virtual void initialize();
	virtual void finishSpecific();

private:
	int totNodes ;
	int* rxBeacons ;
	int* txBeacons ;
	int* cfeStats ;

	int appRx ;
	int appTx ;
	int appDupRx ;

	long round ;
	MetricMapByRound mMapByRound ;
	PhySampleByRound pSampleByRound ;
	PhySampleByRound pSampleRxByRound ;
	ActProbByRound aProbByRound ;

	void init(metricMap* mm) ;
	void printFile(const string& outputFile, const string& filename);
};
