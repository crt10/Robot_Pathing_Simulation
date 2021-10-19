#include "systemc.h"

#define OBSTACLE_SPEED 4
#define ROBOT_SPEED 2

template<int map_size_x, int map_size_y, int grid_size, int num_of_robots, int num_of_obstacles> class processing:public sc_module {
	public:
		//PORTS
		sc_in<bool> clock;
		sc_in<bool> tx_ack[num_of_robots];
		sc_out<bool> tx_flag[num_of_robots];
		sc_out<sc_uint<16> > tx_data[num_of_robots];
		sc_out<bool> rx_ack[num_of_robots];
		sc_in<bool> rx_flag[num_of_robots];
		sc_in<sc_uint<16> > rx_data[num_of_robots];
		
		//CONSTRUCTOR
		SC_HAS_PROCESS(processing);
		
		processing(sc_module_name name, const int* map_ptr, const int* robot_path_ptr, const int* obstacle_path_ptr):
		sc_module(name), _map_ptr(map_ptr), _robot_path_ptr(robot_path_ptr), _obstacle_path_ptr(obstacle_path_ptr) {
			SC_METHOD(prc_update);
			sensitive << clock.pos();
			
			SC_THREAD(prc_tx);

			for (int x = 0; x < map_size_x; x++) {
				for (int y = 0; y < map_size_y; y++) {
					_map_data[x][y] = *(_map_ptr + y*map_size_x + x);	//store map data locally
				}
			}
			for (int i = 0; i < num_of_obstacles; i++) {		//initialize all obstacles
				_obstacles[i].status = 0;
				_obstacles[i].position_x = grid_size/2;
				_obstacles[i].position_y = grid_size/2;
				_obstacles[i].speed = OBSTACLE_SPEED;
				for (int o = 0; o < 23; o++) {
					_obstacles[i].path[o] = *(_obstacle_path_ptr + i*23 + o);
				}
				_obstacles[i].current_grid = _obstacles[i].path[0];
				_obstacles[i].next_grid = _obstacles[i].path[1];
				for (int x = 0; x < map_size_x; x++) {
					for (int y = 0; y < map_size_y; y++) {
						//store the map xy coordinate of the grids
						if (_obstacles[i].current_grid == _map_data[x][y]) {
							_obstacles[i].current_grid_map_x = x;
							_obstacles[i].current_grid_map_y = y;
						}
						if (_obstacles[i].next_grid == _map_data[x][y]) {
							_obstacles[i].next_grid_map_x = x;
							_obstacles[i].next_grid_map_y = y;
						}
					}
				}
			}
			for (int i = 0; i < num_of_robots; i++) {			//initialize all robots
				for (int o = 0; o < 17; o++) {
					_robot_path[i][o] = *(_robot_path_ptr + i*17 + o);	//store robot path data locally (hard-coded for phase 1)
					if (*(_robot_path_ptr + i*17 + o) == -1) {
						break;
					}
				}
				
				_robots[i].status = 3;							//init robots in STOPPED
				_robots[i].position_x = grid_size/2;			//init robots to center of grid
				_robots[i].position_y = grid_size/2;
				_robots[i].speed = ROBOT_SPEED;							//init robot speed to 2 (hard-coded for phase 1)
				
				_main_table[i].status = 3;						//init main_table
				_main_table[i].prev_status = 0;
				_main_table[i].current_grid = _robot_path[i][0];
				_main_table[i].next_grid = _robot_path[i][1];
				_main_table[i].speed = ROBOT_SPEED;
				_main_table[i].modified = false;
				for (int x = 0; x < map_size_x; x++) {
					for (int y = 0; y < map_size_y; y++) {
						//store the map xy coordinate of the grids
						if (_main_table[i].current_grid == _map_data[x][y]) {
							_main_table[i].current_grid_map_x = x;
							_main_table[i].current_grid_map_y = y;
						}
						if (_main_table[i].next_grid == _map_data[x][y]) {
							_main_table[i].next_grid_map_x = x;
							_main_table[i].next_grid_map_y = y;
						}
					}
				}

				//modify center position based on next grid
				if (_main_table[i].next_grid_map_x < _main_table[i].current_grid_map_x) {
					_robots[i].position_x += ROBOT_SPEED;
				}
				else if (_main_table[i].next_grid_map_x > _main_table[i].current_grid_map_x) {
					_robots[i].position_x -= ROBOT_SPEED;
				}
				else if (_main_table[i].next_grid_map_y < _main_table[i].current_grid_map_y) {
					_robots[i].position_y += ROBOT_SPEED;
				}
				else if (_main_table[i].next_grid_map_y > _main_table[i].current_grid_map_y) {
					_robots[i].position_y -= ROBOT_SPEED;
				}
				
				_tx_table[i].status = 3;						//init tx_table
				_tx_table[i].modified = false;
				_rx_table[i].status = 3;						//init rx_table
				_rx_table[i].modified = false;
			}
			_tx_counter = 0;
			_rx_counter = 0;
		}

	private:
		//LOCAL VAR
		typedef struct Robot{
			int status;			//navigation status of robot
			int position_x;		//positions of robot X
			int position_y;		//positions of robot Y
			int speed;			//speed of robot
		}Robot;
		
		typedef struct Robot_Status {	//NOTE: Used for rx and tx tables
			int status;			//navigation status of robot
			bool modified;		//whether status has been modified
		}Robot_Status;
		
		typedef struct Robot_Main_Status {
			int status;
			int prev_status;
			int current_grid;
			int current_grid_map_x;
			int current_grid_map_y;
			int next_grid;
			int next_grid_map_x;
			int next_grid_map_y;
			int speed;
			bool modified;
		}Robot_Main_Status;

		typedef struct Obstacle{
			int status;
			int position_x;
			int position_y;
			int speed;
			int current_grid;
			int current_grid_map_x;
			int current_grid_map_y;
			int next_grid;
			int next_grid_map_x;
			int next_grid_map_y;
			int path[23];
		}Obstacle;
		
		const int* _map_ptr;						//pointer to map data
		int _map_data[map_size_x][map_size_y];		//locally stored map data
		const int* _robot_path_ptr;					//pointer to robot path data
		int _robot_path[num_of_robots][17];			//parameterized robots path (hard-coded for phase 1)
		const int* _obstacle_path_ptr;				//pointer to obstacle path data
		Obstacle _obstacles[num_of_obstacles];		//array of all obstacles
		Robot _robots[num_of_robots];				//array of all robots
		Robot_Main_Status _main_table[num_of_robots];
		
		int _tx_counter;
		int _rx_counter;
		Robot_Status _tx_table[num_of_robots];
		Robot_Status _rx_table[num_of_robots];
		sc_event tx_signal;
		sc_event rx_signal;

		int _clock_count = -1;
		
		//PROCESS
		void prc_update() {
			//DEBUG
			// cout << "TIME " << sc_time_stamp() << endl;
			// cout << "x: " << _robots[2].position_x << endl;
			// cout << "y: " << _robots[2].position_y << endl;

			while (_rx_counter > 0) {					//if recieved data
				for (int i = 0; i < num_of_robots; i++) {	//loop through rx table
					if (_rx_table[i].modified) {
						_main_table[i].status = _rx_table[i].status;	//update main_table status
						_rx_counter--;
					}
				}
			}
			
			
			for (int i = 0; i < num_of_obstacles; i++) {
				bool obstacle_moved = obstacle_move(i);
				switch (_obstacles[i].status) {
					case 0:								//STATE: RESUME
						if (obstacle_moved) {
							_obstacles[i].status = 2;	//update status to crossed
						}
						break;
					case 2:								//STATE: CROSSED
						if (obstacle_moved) {
							_obstacles[i].status = 0;	//update status to resume
						}
						break;
					default:
						break;
				}
			}
			for (int i = 0; i < num_of_robots; i++) {
				bool robot_moved = robot_move(i);
				switch (_main_table[i].status) {
					case 0:								//STATE: RESUME
						if (robot_moved) {
							if (_robots[i].position_x <= grid_size/10 ||
							_robots[i].position_x >= grid_size - (grid_size/10) ||
							_robots[i].position_y <= grid_size/10 ||
							_robots[i].position_y >= grid_size - (grid_size/10)) {
								_main_table[i].prev_status = _main_table[i].status;
								_main_table[i].status = 1;	//update status to crossing
								_robots[i].status = 1;
								_tx_table[i].status = 1;
								_tx_table[i].modified = true;
								_tx_counter++;
							}
						}
						else {
							_main_table[i].prev_status = _main_table[i].status;
							_main_table[i].status = 3;		//update status to stopped
							_robots[i].status = 3;
							_tx_table[i].status = 3;
							_tx_table[i].modified = true;
							_tx_counter++;
						}
						break;
					case 1:								//STATE: CROSSING
						if (robot_moved) {
							if (_main_table[i].modified) {
								_main_table[i].prev_status = _main_table[i].status;
								_main_table[i].modified = 0;
								_main_table[i].status = 2;	//update status to crossed
								_robots[i].status = 2;
								_tx_table[i].status = 2;
								_tx_table[i].modified = true;
								_tx_counter++;
							}
						}
						else {
							_main_table[i].prev_status = _main_table[i].status;
							_main_table[i].status = 3;		//update status to stopped
							_robots[i].status = 3;
							_tx_table[i].status = 3;
							_tx_table[i].modified = true;
							_tx_counter++;
						}
						break;
					case 2:								//STATE: CROSSED
						if (robot_moved) {
							if (_robots[i].position_x == grid_size/2 &&
								_robots[i].position_y == grid_size/2) {
									_main_table[i].prev_status = _main_table[i].status;
									_main_table[i].status = 0;	//update status to resume
									_robots[i].status = 0;
									_tx_table[i].status = 0;
									_tx_table[i].modified = true;
									_tx_counter++;
							}
						}
						else {
							_main_table[i].prev_status = _main_table[i].status;
							_main_table[i].status = 3;			//update status to stopped
							_robots[i].status = 3;
							_tx_table[i].status = 3;
							_tx_table[i].modified = true;
							_tx_counter++;
						}
						break;
					case 3:								//STATE: STOPPED
						if (robot_moved) {
							_main_table[i].status = _main_table[i].prev_status;	//update status to previous status
							_robots[i].status = _main_table[i].prev_status;
							_tx_table[i].status = _main_table[i].prev_status;
							_tx_table[i].modified = true;
							_tx_counter++;
							_main_table[i].prev_status = 3;
						}
						break;
					default:
						break;
				}
			}

			if (_tx_counter > 0) {
				tx_signal.notify(SC_ZERO_TIME);
				cout << endl;
			}
			if (_clock_count++ == 99) {
				_clock_count = 0;
				cout << "TIME: " << sc_time_stamp() << endl;
				for (int i = 0; i < num_of_robots; i++) {
					cout << "Robot " << i+1 << " Current Grid: " 
						 << _main_table[i].current_grid
						 << " | Next Grid: "
						 << _main_table[i].next_grid
						 << " | Position in grid: ("
						 << _robots[i].position_x << ", "
						 << _robots[i].position_y << ")"
						 << endl;
				}
				for (int i = 0; i < num_of_obstacles; i++) {
					cout << "Obstacle " << i+1 << " Current Grid: " 
						 << _obstacles[i].current_grid
						 << " | Next Grid: "
						 << _obstacles[i].next_grid
						 << " | Position in grid: ("
						 << _obstacles[i].position_x << ", "
						 << _obstacles[i].position_y << ")"
						 << endl;
				}
			}
		}
		
		void prc_tx() {
			while (1) {
				wait(tx_signal);
				while (_tx_counter > 0) {					//if recieved data
					for (int i = 0; i < num_of_robots; i++) {	//loop through tx table
						if (_tx_table[i].modified) {
							tx_flag[i] = 1;						//set tx flag
							tx_data[i] = _tx_table[i].status;	//write data to tx channel
							wait(SC_ZERO_TIME);					//wait for ack bit from robot
							wait(SC_ZERO_TIME);
							if (tx_ack[i] == 1) {
								_tx_table[i].modified = false;	//if ack was found,
								_tx_counter--;					//decrement tx_counter
							}
							tx_flag[i] = 0;						//clear tx flag
							wait(SC_ZERO_TIME);
							break;					//note: if ack was not set,
						}							//process will keep sending until successful
					}
				}
			}
		}
		
		bool robot_move(int robot) {
			//find the robot that is occupying the next grid (if there is one)
			int robot2;
			for (robot2 = 0; robot2 < num_of_robots; robot2++) {
				if (_main_table[robot2].current_grid == _main_table[robot].next_grid) {
					break;
				}
			}
			int obstacle;
			for (obstacle = 0; obstacle < num_of_obstacles; obstacle++) {
				if (_obstacles[obstacle].current_grid == _main_table[robot].next_grid ||
					_obstacles[obstacle].current_grid == _main_table[robot].current_grid) {
					break;
				}
			}
			
			//DEBUG
			// if (robot == 2) {
			// 	cout << "current grid " << _main_table[2].current_grid << endl;
			// 	cout << "next grid " << _main_table[2].next_grid << endl;
			// 	cout << "current x " << _main_table[2].current_grid_map_x << endl;
			// 	cout << "current y " << _main_table[2].current_grid_map_y << endl;
			// 	cout << "next x " << _main_table[2].next_grid_map_x << endl;
			// 	cout << "next y " << _main_table[2].next_grid_map_y << endl;
			// }

			if (_main_table[robot].status != 2) {		//if robot is not CROSSED, we need to move towards the middle, regardles of next grid
				//MOVE LEFT
				if (_main_table[robot].next_grid_map_x < _main_table[robot].current_grid_map_x &&
					obstacle == num_of_obstacles) {
					//check if robot is about to cross grids
					if (_robots[robot].position_x - _robots[robot].speed < 0) {
						if (robot2 == num_of_robots) {
							bool grid_updated = table_update_grid(robot);
							if (grid_updated) {
								_robots[robot].position_x -= _robots[robot].speed;
								_robots[robot].position_x += grid_size;
								return true;								//return true if grid is not occupied and grid updated
							}
							else {
								return false;								//return false if grid not updated (end of path reached)
							}
						}
						else {
							return false;								//return false if grid is occupied
						}
					}
					else {
						//move the robot if robot is staying inside grid
						_robots[robot].position_x -= _robots[robot].speed;
						return true;
					}
				}
				//MOVE RIGHT
				else if (_main_table[robot].next_grid_map_x > _main_table[robot].current_grid_map_x &&
						 obstacle == num_of_obstacles) {
					//check if robot is about to cross grids
					if (_robots[robot].position_x + _robots[robot].speed > grid_size) {
						if (robot2 == num_of_robots) {
							bool grid_updated = table_update_grid(robot);
							if (grid_updated) {
								_robots[robot].position_x += _robots[robot].speed;
								_robots[robot].position_x -= grid_size;
								return true;								//return true if grid is not occupied and grid updated
							}
							else {
								return false;								//return false if grid not updated (end of path reached)
							}
						}
						else {
							return false;								//return false if grid is occupied
						}
					}		
					else {
						//move the robot if robot is staying inside grid
						_robots[robot].position_x += _robots[robot].speed;
						return true;
					}		
				}
				//MOVE DOWN
				else if (_main_table[robot].next_grid_map_y < _main_table[robot].current_grid_map_y &&
						 obstacle == num_of_obstacles) {
					//check if robot is about to cross grids
					if (_robots[robot].position_y - _robots[robot].speed < 0) {
						if (robot2 == num_of_robots) {
							bool grid_updated = table_update_grid(robot);
							if (grid_updated) {
								_robots[robot].position_y -= _robots[robot].speed;
								_robots[robot].position_y += grid_size;
								return true;								//return true if grid is not occupied and grid updated
							}
							else {
								return false;								//return false if grid not updated (end of path reached)
							}
						}
						else {
							return false;								//return false if grid is occupied
						}
					}
					else {
						//move the robot if robot is staying inside grid
						_robots[robot].position_y -= _robots[robot].speed;
						return true;
					}
				}
				//MOVE UP
				else if (_main_table[robot].next_grid_map_y > _main_table[robot].current_grid_map_y &&
						 obstacle == num_of_obstacles) {
					//check if robot is about to cross grids
					if (_robots[robot].position_y + _robots[robot].speed > grid_size) {
						if (robot2 == num_of_robots) {
							bool grid_updated = table_update_grid(robot);
							if (grid_updated) {
								_robots[robot].position_y += _robots[robot].speed;
								_robots[robot].position_y -= grid_size;
								return true;								//return true if grid is not occupied and grid updated
							}
							else {
								return false;								//return false if grid not updated (end of path reached)
							}
						}
						else {
							return false;								//return false if grid is occupied
						}
					}
					else {
						//move the robot if robot is staying inside grid
						_robots[robot].position_y += _robots[robot].speed;
						return true;
					}
				}
				//DONT MOVE (robot has reached end of path) or (obstacle detected)
				else {
					return false;
				}
			}
			else {
				if (obstacle == num_of_obstacles) {
					if (_robots[robot].position_x < grid_size/2) {
						_robots[robot].position_x += _robots[robot].speed;
					}
					else if (_robots[robot].position_x > grid_size/2) {
						_robots[robot].position_x -= _robots[robot].speed;
					}
					else if (_robots[robot].position_y < grid_size/2) {
						_robots[robot].position_y += _robots[robot].speed;
					}
					else if (_robots[robot].position_y > grid_size/2) {
						_robots[robot].position_y -= _robots[robot].speed;
					}
					if (_main_table[robot].next_grid == -1 &&
						_robots[robot].position_x == grid_size/2 &&
						_robots[robot].position_y == grid_size/2) {	//Edge case for when robot reaches last grid in path
						return false;
					}
					else {
						return true;
					}
				}
				else {
					return false;
				}
			}
		}
		
		bool table_update_grid(int robot) {
			int new_next_grid = -1;
			//search for next grid in path
			for (int i = 0; i < 15; i++) {
				if (_robot_path[robot][i] == _main_table[robot].next_grid) {
					new_next_grid = _robot_path[robot][i+1];
					break;
				}
			}
			
			//write new grids into table (only if robot hasn't reached end of path)
			if (new_next_grid != -1) {
				_main_table[robot].current_grid = _main_table[robot].next_grid;
				_main_table[robot].next_grid = new_next_grid;
				for (int x = 0; x < map_size_x; x++) {
					for (int y = 0; y < map_size_y; y++) {
						//store the map xy coordinate of the grids
						if (_main_table[robot].current_grid == _map_data[x][y]) {
							_main_table[robot].current_grid_map_x = x;
							_main_table[robot].current_grid_map_y = y;
						}
						if (_main_table[robot].next_grid == _map_data[x][y]) {
							_main_table[robot].next_grid_map_x = x;
							_main_table[robot].next_grid_map_y = y;
						}
					}
				}
				_main_table[robot].modified = true;
				return true;
			}
			else {
				_main_table[robot].current_grid_map_x = _main_table[robot].next_grid_map_x ;
				_main_table[robot].current_grid_map_y = _main_table[robot].next_grid_map_y;
				_main_table[robot].current_grid = _main_table[robot].next_grid;
				_main_table[robot].next_grid = -1;
				if (_main_table[robot].status == 1) {		//Edge case for when robot reaches last grid in path
					_main_table[robot].modified = 1;
					return true;
				}
				else {
					return false;
				}
			}
		}

		bool obstacle_move(int obstacle) {
			if (_obstacles[obstacle].status != 2) {		//if obstacle is not CROSSED, we need to move towards the middle, regardles of next grid
				//MOVE LEFT
				if (_obstacles[obstacle].next_grid_map_x < _obstacles[obstacle].current_grid_map_x) {
					//check if obstacle is about to cross grids
					if (_obstacles[obstacle].position_x - _obstacles[obstacle].speed < 0) {
						//move the obstacle and update the grid if crossing
						obstacle_update_grid(obstacle);
						_obstacles[obstacle].position_x -= _obstacles[obstacle].speed;
						_obstacles[obstacle].position_x += grid_size;
						return true;
					}
					else {
						//move the obstalce if obstacle is staying inside grid
						_obstacles[obstacle].position_x -= _obstacles[obstacle].speed;
						return false;
					}
				}
				//MOVE RIGHT
				else if (_obstacles[obstacle].next_grid_map_x > _obstacles[obstacle].current_grid_map_x) {
					//check if obstacle is about to cross grids
					if (_obstacles[obstacle].position_x + _obstacles[obstacle].speed > grid_size) {
						//move the obstacle and update the grid if crossing
						obstacle_update_grid(obstacle);
						_obstacles[obstacle].position_x += _obstacles[obstacle].speed;
						_obstacles[obstacle].position_x -= grid_size;
						return true;
					}
					else {
						//move the obstalce if obstacle is staying inside grid
						_obstacles[obstacle].position_x += _obstacles[obstacle].speed;
						return false;
					}
				}
				//MOVE DOWN
				else if (_obstacles[obstacle].next_grid_map_y < _obstacles[obstacle].current_grid_map_y) {
					//check if obstacle is about to cross grids
					if (_obstacles[obstacle].position_y - _obstacles[obstacle].speed < 0) {
						//move the obstacle and update the grid if crossing
						obstacle_update_grid(obstacle);
						_obstacles[obstacle].position_y -= _obstacles[obstacle].speed;
						_obstacles[obstacle].position_y += grid_size;
						return true;
					}
					else {
						//move the obstalce if obstacle is staying inside grid
						_obstacles[obstacle].position_y -= _obstacles[obstacle].speed;
						return false;
					}
				}
				//MOVE UP
				else if (_obstacles[obstacle].next_grid_map_y > _obstacles[obstacle].current_grid_map_y) {
					//check if obstacle is about to cross grids
					if (_obstacles[obstacle].position_y + _obstacles[obstacle].speed > grid_size) {
						//move the obstacle and update the grid if crossing
						obstacle_update_grid(obstacle);
						_obstacles[obstacle].position_y += _obstacles[obstacle].speed;
						_obstacles[obstacle].position_y -= grid_size;
						return true;
					}
					else {
						//move the obstalce if obstacle is staying inside grid
						_obstacles[obstacle].position_y += _obstacles[obstacle].speed;
						return false;
					}
				}
				//DONT MOVE
				else {
					return false;
				}
			}
			else {
				if (_obstacles[obstacle].position_x < grid_size/2) {
					_obstacles[obstacle].position_x += _obstacles[obstacle].speed;
				}
				else if (_obstacles[obstacle].position_x > grid_size/2) {
					_obstacles[obstacle].position_x -= _obstacles[obstacle].speed;
				}
				else if (_obstacles[obstacle].position_y < grid_size/2) {
					_obstacles[obstacle].position_y += _obstacles[obstacle].speed;
				}
				else if (_obstacles[obstacle].position_y > grid_size/2) {
					_obstacles[obstacle].position_y -= _obstacles[obstacle].speed;
				}
				if (_obstacles[obstacle].position_x == grid_size/2 &&
					_obstacles[obstacle].position_y == grid_size/2) {
					return true;
				}
				else {
					return false;
				}
			}
		}
		
		void obstacle_update_grid(int obstacle) {
			int new_next_grid = -1;
			//search for next grid in path
			for (int i = 0; i < 23; i++) {
				if (_obstacles[obstacle].path[i] == _obstacles[obstacle].next_grid) {
					new_next_grid = _obstacles[obstacle].path[i+1];
					break;
				}
			}

			_obstacles[obstacle].current_grid = _obstacles[obstacle].next_grid;
			_obstacles[obstacle].next_grid = new_next_grid;
			for (int x = 0; x < map_size_x; x++) {
				for (int y = 0; y < map_size_y; y++) {
					//store the map xy coordinate of the grids
					if (_obstacles[obstacle].current_grid == _map_data[x][y]) {
						_obstacles[obstacle].current_grid_map_x = x;
						_obstacles[obstacle].current_grid_map_y = y;
					}
					if (_obstacles[obstacle].next_grid == _map_data[x][y]) {
						_obstacles[obstacle].next_grid_map_x = x;
						_obstacles[obstacle].next_grid_map_y = y;
					}
				}
			}
		}

};
