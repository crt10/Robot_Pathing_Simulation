#include "systemc.h"

template<int map_size_x, int map_size_y, int num_of_robots> class server:public sc_module {
	public:
		//PORTS
		sc_in<bool> clock;
		sc_in<bool> tx_ack[num_of_robots];
		sc_out<bool> tx_flag[num_of_robots];
		sc_out<sc_uint<16> > tx_data[num_of_robots];
		sc_out<bool> rx_ack[num_of_robots];
		sc_in<bool> rx_flag[num_of_robots];
		sc_in<sc_uint<16> > rx_data[num_of_robots];
		sc_fifo_out<int> fifo_data[num_of_robots];
		
		//CONSTRUCTOR
		SC_HAS_PROCESS(server);
		
		server(sc_module_name name, const int* map_ptr, const int* robot_path_ptr):
		sc_module(name), _map_ptr(map_ptr), _robot_path_ptr(robot_path_ptr) {
			SC_METHOD(prc_update);
			sensitive << clock.pos();
			
			SC_THREAD(prc_tx);
			sensitive << tx_ack[0].pos() << tx_ack[1].pos() << tx_ack[2].pos() << tx_ack[3].pos();
			
			SC_THREAD(prc_rx);
			sensitive << rx_flag[0].pos() << rx_flag[1].pos() << rx_flag[2].pos() << rx_flag[3].pos();

			for (int x = 0; x < map_size_x; x++) {
				for (int y = 0; y < map_size_y; y++) {
					_map_data[x][y] = *(_map_ptr + y*map_size_x + x);	//store map data locally
				}
			}
			for (int i = 0; i < num_of_robots; i++) {			//initialize all robots
				for (int o = 0; o < 23; o++) {
					_robot_path[i][o] = *(_robot_path_ptr + i*23 + o);	//store robot path data locally
					if (*(_robot_path_ptr + i*23 + o) == -1) {
						break;
					}
				}
				
				_tx_table[i].status = 7;						//init tx_table
				_tx_table[i].modified = false;
				_rx_table[i].status = 7;						//init rx_table
				_rx_table[i].modified = false;
				
				_main_table[i].status = 5;						//init main_table
				_main_table[i].current_grid = _robot_path[i][0];
				_main_table[i].next_grid = _robot_path[i][1];
				_main_table[i].speed = 0;
			}
			_tx_counter = 0;
			_rx_counter = 0;
		}

	private:
		//LOCAL VAR
		typedef struct Robot_Status {	//NOTE: Used for rx and tx tables
			int status;			//navigation status of robot
			bool modified;		//whether status has been modified
		}Robot_Status;
		
		typedef struct Robot_Main_Status {
			int status;
			int current_grid;
			int next_grid;
			int speed;
		}Robot_Main_Status;
		
		const int* _map_ptr;						//pointer to map data
		int _map_data[map_size_x][map_size_y];		//locally stored map data
		const int* _robot_path_ptr;					//pointer to robot path data
		int _robot_path[num_of_robots][23];			//parameterized robots path (hard-coded for phase 1)
		Robot_Main_Status _main_table[num_of_robots];
		
		int _tx_counter;
		int _rx_counter;
		Robot_Status _tx_table[num_of_robots];
		Robot_Status _rx_table[num_of_robots];
		sc_event tx_signal;

		int _clock_count = -1;
		
		void prc_tx() {
			while (1) {
				wait(tx_signal);
				while (_tx_counter > 0) {					//if recieved data
					for (int i = 0; i < num_of_robots; i++) {	//loop through tx table
						if (_tx_table[i].modified) {
							tx_flag[i] = 1;						//set tx flag
							tx_data[i] = _tx_table[i].status;	//write data to tx channel
							wait();								//wait for ack bit from robot
							if (tx_ack[i] == 1) {
								_tx_table[i].modified = false;	//if ack was found,
								_tx_counter--;					//decrement tx_counter
							}
							tx_flag[i] = 0;						//clear tx flag
							wait(SC_ZERO_TIME);
							break;					//note: if ack was not recieved,
						}							//process will keep sending until successful
					}
				}
			}
		}
		
		void prc_rx() {
			while(1) {
				wait();
				for (int i = 0; i < num_of_robots; i++) {
					if (rx_flag[i] == 1) {
						rx_ack[i] = 1;								//send ack bit
						_rx_table[i].status = rx_data[i].read();	//update rx table
						_rx_table[i].modified = 1;
						wait(SC_ZERO_TIME);
						rx_ack[i] = 0;
						_rx_counter++;
					}
				}
			}
		}
		
		void prc_update() {			
			while (_rx_counter > 0) {					//if recieved data
				for (int i = 0; i < num_of_robots; i++) {	//loop through rx table
					if (_rx_table[i].modified) {
						//cout <<  sc_time_stamp() << i << "in" << _main_table[i].status << endl;
						if (_main_table[i].status != 5) {
							switch(_rx_table[i].status) {
								case 0:
								case 3:
									_main_table[i].status = 3;
									break;
								case 1:
									if (robot_move(i)) {
										_main_table[i].status = 0;
										_tx_table[i].status = 9;
										_tx_table[i].modified = 1;
										_tx_counter++;
									}
									else {
										_main_table[i].status = 3;
										_tx_table[i].status = 8;
										_tx_table[i].modified = 1;
										_tx_counter++;
									}
									break;
								case 2:
									if (robot_move(i)) {
										_main_table[i].status = 0;
										_tx_table[i].status = 5;
										_tx_table[i].modified = 1;
										_tx_counter++;
									}
									else {
										_main_table[i].status = 3;
										_tx_table[i].status = 7;
										_tx_table[i].modified = 1;
										_tx_counter++;
									}
									break;
								case 4:
									_main_table[i].status = 2;
									_main_table[i].current_grid = _main_table[i].next_grid;
									_main_table[i].next_grid = next_grid(i);
									if (_main_table[i].next_grid == -1) {
										_main_table[i].status = 5;
										_tx_table[i].status = 7;
										_tx_table[i].modified = 1;
										_tx_counter++;
									}
									break;
								default:
									break;
							}
						}
						_rx_counter--;
						_rx_table[i].modified = 0;
					}
				}
			}
			
			for (int i = 0; i < num_of_robots; i++) {
				bool robot_moved = robot_move(i);
				//cout <<  sc_time_stamp() << i << "inloop" << _tx_table[i].modified << endl;
				if (_tx_table[i].modified == 0) {
					switch (_main_table[i].status) {
						case 0:								//STATE: RESUME
						case 2:								//STATE: CROSSED
							if (!robot_moved) {
								if (_main_table[i].speed == 2) {
									fifo_data[i].write(1);
									fifo_data[i].write(0);
									_tx_table[i].status = 10;
									_tx_table[i].modified = 1;
									_tx_counter++;
									_main_table[i].speed = 1;
								}
								else if (_main_table[i].speed == 1) {
									_tx_table[i].status = 10;
									_tx_table[i].modified = 1;
									_tx_counter++;
									_main_table[i].speed = 0;
								}
								else {
									_main_table[i].status = 3;
									_tx_table[i].status = 8;
									_tx_table[i].modified = 1;
									_tx_counter++;
								}
							}
							else {
								_main_table[i].status = 0;
								if (_main_table[i].speed == 0) {
									fifo_data[i].write(1);
									fifo_data[i].write(2);
									_tx_table[i].status = 10;
									_tx_table[i].modified = 1;
									_tx_counter++;
									_main_table[i].speed = 1;
								}
								else if (_main_table[i].speed == 1) {
									_tx_table[i].status = 10;
									_tx_table[i].modified = 1;
									_tx_counter++;
									_main_table[i].speed = 2;
								}
							}
							break;
						case 3:								//STATE: STOPPED
							if (robot_moved) {
								_main_table[i].status = 0;
								_tx_table[i].status = 9;
								_tx_table[i].modified = 1;
								_tx_counter++;
							}
							break;
						case 6:								//Special case for start up
							if (robot_moved) {
								_main_table[i].status = 0;
								_tx_table[i].status = 6;
								_tx_table[i].modified = 1;
								_tx_counter++;
							}
							break;
						default:
							break;
					}
				}
			}

			_clock_count++;
			if (_clock_count == 101) {
				send_path(0);
				_main_table[0].status = 6;
			}
			else if (_clock_count == 501) {
				send_path(1);
				_main_table[1].status = 6;
			}
			else if (_clock_count == 701) {
				send_path(2);
				_main_table[2].status = 6;
			}
			else if (_clock_count == 201) {
				send_path(3);
				_main_table[3].status = 6;
			}
			
			if (_tx_counter > 0) {
				tx_signal.notify(SC_ZERO_TIME);
				cout << endl;
			}
		}
		
		int next_grid(int robot) {
			int new_next_grid = -1;
			//search for next grid in path
			for (int i = 0; i < 23; i++) {
				if (_robot_path[robot][i] == _main_table[robot].next_grid) {
					new_next_grid = _robot_path[robot][i+1];
					break;
				}
			}
			return new_next_grid;
		}
		
		bool robot_move(int robot) {
			//find the robot that is occupying the next grid (if there is one)
			int robot2;
			for (robot2 = 0; robot2 < num_of_robots; robot2++) {
				if (_main_table[robot2].current_grid == _main_table[robot].next_grid &&
					_main_table[robot2].status != 6 && _main_table[robot2].status != 5) {
					break;
				}
			}
			
			if (robot2 == num_of_robots) {
				return true;
			}
			else {
				return false;
			}
		}
		
		void send_path(int robot) {
			_tx_table[robot].status = 11;
			_tx_table[robot].modified = 1;
			_tx_counter++;
			for (int i = 0; i < 23; i++) {
				fifo_data[robot].write(_robot_path[robot][i]);
				if (_robot_path[robot][i] == -1) {
					break;
				}
			}
		}
};