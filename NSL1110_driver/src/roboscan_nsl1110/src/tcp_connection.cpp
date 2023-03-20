#include <iostream>
#include "tcp_connection.hpp"

using boost::asio::ip::tcp;


namespace nanosys {

typedef std::vector<uint8_t> Packet;

TcpConnection::TcpConnection(boost::asio::io_service& ioService)
  : resolver(ioService), socket(ioService), state(STATE_DISCONNECTED) 
{

}

TcpConnection::~TcpConnection() {
  try {
    disconnect();
  } catch (boost::system::system_error &e) {
    std::cerr << e.what() << std::endl;
  }
}

bool TcpConnection::sendCommand(uint8_t *data, int data_len) 
{
	connect();

	std::ostringstream os;

	for (int i = 0; i < data_len; ++i) {
		os << data[i];
	}

//	printf("sendCommand len = %d: %s\n", data_len, (char *)data);

	boost::system::error_code error;
	socket.write_some(boost::asio::buffer(os.str(), os.tellp()), error);
	if (error) {
//		throw boost::system::system_error(error);
		return false;
	}
	
	waitAck();

	disconnect();

	usleep(30000);

	return true;  
}


bool TcpConnection::sendCommand(uint8_t *data, int data_len, Packet &databuf ) 
{
	connect();

	std::ostringstream os;

	for (int i = 0; i < data_len; ++i) {
		os << data[i];
	}

//	printf("sendCommand len = %d: %s\n", data_len, (char *)data);

	boost::system::error_code error;
	socket.write_some(boost::asio::buffer(os.str(), os.tellp()), error);
	if (error) {
//		throw boost::system::system_error(error);
		return false;
	}
	
	bool bRet = waitData(databuf);

	disconnect();

  	return bRet;
}


void TcpConnection::connect() {
  if (isConnected()) return;

  updateState(STATE_CONNECTING);
  tcp::resolver::query query(HOST, PORT);
  tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
  tcp::resolver::iterator end;

  boost::system::error_code error = boost::asio::error::host_not_found;
  while (error && endpoint_iterator != end) {
    socket.close();
    socket.connect(*endpoint_iterator++, error);
  }
  if (error) {
    throw::boost::system::system_error(error);
  }
  updateState(STATE_CONNECTED);
}

void TcpConnection::disconnect() {
  if (isDisconnected()) return;

  updateState(STATE_CLOSING);

  boost::system::error_code error;
  socket.shutdown(boost::asio::ip::tcp::socket::shutdown_both, error);
  if (error) {
    revertState();
    throw boost::system::system_error(error);
  }
  updateState(STATE_DISCONNECTED);
}

void TcpConnection::waitAck() {
	Packet buf(ACK_BUF_SIZE);
	boost::system::error_code error;

	this->updateState(STATE_WAIT_ACK);
	socket.read_some(boost::asio::buffer(buf), error);

	if (error) {
		//throw boost::system::system_error(error);
		printf("wait ack error :: %d\n", error.value());
	}
	else{
//		printf("ack :: %s\n", reinterpret_cast<char*>(buf.data()));
	}

	this->revertState();
}

bool TcpConnection::waitData(Packet &databuf) 
{
	bool bRxOk = false;
	boost::system::error_code error;
	int totalLen = 0;
	//static Packet databuf(IMAGE_DATA_SIZE);
	static Packet rxbuf(320*240*4*2);
	
	this->updateState(STATE_WAIT_ACK);


	while(true){
		int length = socket.read_some(boost::asio::buffer(rxbuf), error);

		if( length > 0 ){
			std::copy_n( rxbuf.begin(), length, databuf.begin() + totalLen );
			totalLen += length;
		}

		if (error || length == 0 ) {

			if( error.value() > 0 && boost::asio::error::eof != error ){
				printf("wait data error :: %d[%s] eof = %d\n", error.value(), error.message().c_str(), boost::asio::error::eof);
			}
			else if(totalLen == 153600 || totalLen == 307200 || totalLen == 614400){
				bRxOk = true;
			}
			else{
				printf("rx Failed len = %d length = %d %d[%s]\n", totalLen, length, error.value(), error.message().c_str());
			}
			
			break;
		}
	}

	this->revertState();

	return bRxOk;
}



void TcpConnection::updateState(State state_) const {
  previousState = state;
  state = state_;
}

void TcpConnection::revertState() const {
  state = previousState;
}

bool TcpConnection::isConnected() const {
  return state == STATE_CONNECTED;
}

bool TcpConnection::isDisconnected() const {
  return state == STATE_DISCONNECTED;
}

} // end namespace nanosys
