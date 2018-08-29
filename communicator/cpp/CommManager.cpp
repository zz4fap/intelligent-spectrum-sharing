/*
 * CommManager.cpp
 *
 *  Created on: 17 Mar 2017
 *      Author: mcamelo
 */
#include <iostream>
#include <cstring>
#include <sstream>
#include <fstream>

#include "logging/Logger.h"
#include "CommManager.h"
#include "interf.pb.h"
#include "utils.h"

using namespace std;

namespace communicator {

static string createUniqueFile(MODULE ownMODULE, MODULE otherMODULE, bool push){
	std::stringstream ss;
	ss << IPCDIR;
	if(push){
		ss << ownMODULE << "_" << otherMODULE;
	}else{
		ss << otherMODULE << "_" << ownMODULE;
	}
	return ss.str();
}

CommManager::CommManager(MODULE module_owner, MODULE other): own_module(module_owner), other_module(other),
				context(2),
				socketPull(context, ZMQ_PULL), socketPush(context, ZMQ_PUSH)
{

	if (MODULE_IsValid(own_module) && MODULE_IsValid(other_module)){
		const std::string nameOwn = communicator::MODULE_Name(own_module);
		const std::string nameOther = communicator::MODULE_Name(other_module);

		const std::string pushFile = createUniqueFile(module_owner, other, true);
		const std::string pullFile = createUniqueFile(module_owner, other, false);

		//TOUCH FILE
		ofstream{pushFile};
		ofstream{pullFile};
		std::string ipcPush = "ipc://" + pushFile;
		std::string ipcPull = "ipc://" + pullFile;

		Logger::log_trace<CommManager>("Bind push socket from {0} to {1}", nameOwn, nameOther);
		socketPush.bind(ipcPush);

		Logger::log_trace<CommManager>("Connect pull socket from {0} to {1}", nameOwn, nameOther);
		socketPull.connect(ipcPull);

		Logger::log_info<CommManager>("Communicator manager started from {0} to {1}", nameOwn, nameOther);
	}
	else{
		Logger::log_error<CommManager>("Manager could not start communicator: Unknown MODULE. See {0}:{1}", __FILE__, __LINE__);
	}
}

CommManager::~CommManager(){
	const std::string nameOwn = communicator::MODULE_Name(own_module);
	const std::string nameOther = communicator::MODULE_Name(other_module);

	Logger::log_trace<CommManager>("Close push socket from {0} to {1}", nameOwn, nameOther);
	int linger = 0;
	socketPush.setsockopt(ZMQ_LINGER, &linger, sizeof(linger));
	socketPush.close();
	Logger::log_trace<CommManager>("Close pull socket from {0} to {1}", nameOwn, nameOther);
	socketPull.setsockopt(ZMQ_LINGER, &linger, sizeof(linger));
	socketPull.close();

	//Logger::log_trace<CommManager>("Close context from {0} to {1}", nameOwn, nameOther);
	//context.close();

	Logger::log_info<CommManager>("Closed CommManager between {0} and {1}", nameOwn, nameOther);
}


bool CommManager::receive(Message& result, long int timeout){
	zmq::message_t message;
//	int more = 0;
//	int index = 0;
//    while (1) {
//		//  Process all parts of the message
//		socketPair.recv(&message);
//		size_t more_size = sizeof (more);
//		socketPair.getsockopt(ZMQ_RCVMORE, &more, &more_size);
//		std::string partOfMessage = std::string(static_cast<char *>(message.data()),message.size());
//
//		//Return message but remove origin from envelop
//		messageContainer.push_back(partOfMessage);
//
//		if (!more)
//			break;      //  Last message part
//		index++;
//    }
	zmq::poll(&pollitem, 1, timeout);
	if(pollitem.revents & ZMQ_POLLIN){
		//socketPull.recv(&message);
		socketPull.recv(&message);
		uint64_t time = clock_get_time_ns();
		uint64_t time2 = clock_get_time_ns();
		Logger::log_trace<CommManager>("!!Receive time = {0}, {1}", time, time2-time);
		std::string data = std::string(static_cast<char *>(message.data()), message.size());
		result.message = deserializedData(data);
	}else{
		return false;
	}

    result.destination = own_module;
    result.source = other_module;

	return true;

}

void CommManager::send(const Message& message){
	Logger::log_trace<CommManager>("Communicator between {0} and {1}. Send message.", MODULE_Name(own_module), MODULE_Name(other_module));

	zmq::message_t msg;
	//Data serialized
	std::string serializedData;
	message.message->SerializeToString(&serializedData);
	const unsigned sz = serializedData.length();
	msg.rebuild(sz);
	memcpy((void *) msg.data (), serializedData.c_str(), sz);
	//socketPush.send(msg, ZMQ_NOBLOCK);
	uint64_t time = clock_get_time_ns();
	socketPush.send(msg, ZMQ_NOBLOCK);
	Logger::log_trace<CommManager>("!!Send time = {0}", time);
}

std::shared_ptr<communicator::Internal> CommManager::deserializedData(std::string &data){
	std::shared_ptr<communicator::Internal> message = std::make_shared<communicator::Internal>();
	message->ParseFromString(data);
	return message;
}

}
