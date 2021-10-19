#include <systemc.h>

static const char* status_names[] = {"RESUMED", "CROSSING", "CROSSED", "STOPPED"};

class robot:public sc_module {
	public:
		//PORTS
		sc_in<bool> clock;
		sc_in<bool> tx_ack;
		sc_out<bool> tx_flag;
		sc_out<sc_uint<16> > tx_data;
		sc_out<bool> rx_ack;
		sc_in<bool> rx_flag;
		sc_in<sc_uint<16> > rx_data;
		
		//CONSTRUCTOR
		SC_HAS_PROCESS(robot);
		
		robot(sc_module_name name):sc_module(name) {
			SC_THREAD(prc_rx);
			sensitive << rx_flag.pos();
			
		}

	private:
		//LOCAL VAR
		int _status;
		
		//PROCESS
		void prc_rx() {
			while(1) {
				wait();
				rx_ack = 1;							//send ack bit
				_status = rx_data.read();			//update robot status
				print_status();						//print status to console
				wait(SC_ZERO_TIME);
				rx_ack = 0;
			}
		}
		
		void print_status() {
			cout << "Time " << sc_time_stamp() << " | "
				 <<	name() << " is now: " << status_names[_status] << endl;
		}
};
