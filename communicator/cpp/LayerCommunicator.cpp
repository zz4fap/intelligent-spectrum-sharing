/*
 * LayerCommunicator.cpp
 *
 *  Created on: 10 Nov 2017
 *      Author: rubenmennes
 */

#include "LayerCommunicator.h"

namespace communicator{

AbstractLayerCommunicator::AbstractLayerCommunicator(MODULE module):
	m_running(true), m_ownModule(module), m_destModule(MODULE_UNKNOWN), m_threads(), m_sending(), m_mutex(), m_cv()
{

}

AbstractLayerCommunicator::~AbstractLayerCommunicator() {

}

void AbstractLayerCommunicator::terminate_communicator() {
	m_running = false;
	m_sending.join();
	for(auto &i: m_threads){
		i.second.receiving.join();
		delete i.second.commManager;
	}
}

void AbstractLayerCommunicator::init(std::vector<MODULE> connectTo){
	for(MODULE i: connectTo){
		if(i == m_ownModule) continue;
		CommManagerThreads comm;
		comm.commManager = new CommManager(m_ownModule, i);
		CommManager* tmp = comm.commManager;
		std::thread receiving([tmp, this] {this->receiving(tmp);});
		comm.receiving.swap(receiving);
		m_threads.emplace(std::make_pair<>(i, std::move(comm)));
		if(connectTo.size() == 1) {
			m_destModule = i;
		} else {
			m_destModule = MODULE_UNKNOWN;
		}
	}
	std::thread sending([this] {this->sending();});
	m_sending.swap(sending);
}

bool AbstractLayerCommunicator::waitForMessage() const{
	std::unique_lock<std::mutex> lck(m_mutex);
	m_cv.wait(lck, [this]{return this->empty();});
	return true;
}

AbstractLayerCommunicator::Queue AbstractLayerCommunicator::filter(const Message& m) const{
	if(m.source == communicator::MODULE_RF_MON) return Low;
	switch (m.message->payload_case()) {
		case communicator::Internal::PAYLOAD_NOT_SET:
			//In case payload not set do not put it in any queue at all
			return None;
		case communicator::Internal::kGet:
		case communicator::Internal::kGetr:
		case communicator::Internal::kSet:
		case communicator::Internal::kSetr:
		case communicator::Internal::kStats:
		case communicator::Internal::kReceiver:
		case communicator::Internal::kSendr:
			return High;
		default:
			return Low;
	}
}

}
