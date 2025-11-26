#include<boost/tokenizer.hpp>
#include <algorithm>    // std::shuffle
#include <random>      // std::default_random_engine
#include <chrono>       // std::chrono::system_clock
#include <string>       // std::to_string
#include"Instance.h"

using std::to_string;

int RANDOM_WALK_STEPS = 100000;

Instance::Instance(const string& map_fname, const string& agent_fname, 
	int num_of_agents, const string& agent_indices, 
	int num_of_rows, int num_of_cols, int num_of_obstacles, int warehouse_width):
	map_fname(map_fname), agent_fname(agent_fname), num_of_agents(num_of_agents),  agent_indices(agent_indices)
{
	bool succ = loadMap();
	if (!succ)
	{
		if (num_of_rows > 0 && num_of_cols > 0 && num_of_obstacles >= 0 && 
			num_of_obstacles < num_of_rows * num_of_cols) // generate random grid
		{
			generateConnectedRandomGrid(num_of_rows, num_of_cols, num_of_obstacles);
			saveMap();
		}
		else
		{
			setError("Map file " + map_fname + " not found and invalid parameters for random generation.");
			return;
		}
	}

	succ = loadAgents();
	if (!succ)
	{
		if (num_of_agents > 0)
		{
			generateRandomAgents(warehouse_width);
			saveAgents();
		}
		else
		{
			setError("Agent file " + agent_fname + " not found and num_of_agents is not positive for random generation.");
			return;
		}
	}

}


int Instance::randomWalk(int curr, int steps) const
{
	for (int walk = 0; walk < steps; walk++)
	{
		list<int> l = getNeighbors(curr);
		vector<int> next_locations(l.cbegin(), l.cend());
		auto rng = std::default_random_engine{};
		std::shuffle(std::begin(next_locations), std::end(next_locations), rng);
		for (int next : next_locations)
		{
			if (validMove(curr, next))
			{
				curr = next;
				break;
			}
		}
	}
	return curr;
}

void Instance::generateRandomAgents(int warehouse_width)
{
	cout << "Generate " << num_of_agents << " random start and goal locations " << endl;
	vector<bool> starts(map_size, false);
	vector<bool> goals(map_size, false);
	start_locations.resize(num_of_agents);
	goal_locations.resize(num_of_agents);

	if (warehouse_width == 0)//Generate agents randomly
	{
		// Choose random start locations
		int k = 0;
		while (k < num_of_agents)
		{
			int x = rand() % num_of_rows, y = rand() % num_of_cols;
			int start = linearizeCoordinate(x, y);
			if (my_map[start] || starts[start])
				continue;
				
			// update start
			start_locations[k] = start;
			starts[start] = true;

			// find goal
			bool flag = false;
			int goal = randomWalk(start, RANDOM_WALK_STEPS);
			while (goals[goal])
				goal = randomWalk(goal, 1);

			//update goal
			goal_locations[k] = goal;
			goals[goal] = true;

			k++;
		}
	}
	else //Generate agents for warehouse scenario
	{
		// Choose random start locations
		int k = 0;
		while (k < num_of_agents)
		{
			int x = rand() % num_of_rows, y = rand() % warehouse_width;
			if (k % 2 == 0)
				y = num_of_cols - y - 1;
			int start = linearizeCoordinate(x, y);
			if (starts[start])
				continue;
			// update start
			start_locations[k] = start;
			starts[start] = true;

			k++;
		}
		// Choose random goal locations
		k = 0;
		while (k < num_of_agents)
		{
			int x = rand() % num_of_rows, y = rand() % warehouse_width;
			if (k % 2 == 1)
				y = num_of_cols - y - 1;
			int goal = linearizeCoordinate(x, y);
			if (goals[goal])
				continue;
			// update goal
			goal_locations[k] = goal;
			goals[goal] = true;
			k++;
		}
	}
}

bool Instance::addObstacle(int obstacle)
{
	if (my_map[obstacle])
		return false;
	my_map[obstacle] = true;
	int obstacle_x = getRowCoordinate(obstacle);
	int obstacle_y = getColCoordinate(obstacle);
	int x[4] = { obstacle_x, obstacle_x + 1, obstacle_x, obstacle_x - 1 };
	int y[4] = { obstacle_y - 1, obstacle_y, obstacle_y + 1, obstacle_y };
	int start = 0;
	int goal = 1;
	while (start < 3 && goal < 4)
	{
		if (x[start] < 0 || x[start] >= num_of_rows || y[start] < 0 || y[start] >= num_of_cols 
			|| my_map[linearizeCoordinate(x[start], y[start])])
			start++;
		else if (goal <= start)
			goal = start + 1;
		else if (x[goal] < 0 || x[goal] >= num_of_rows || y[goal] < 0 || y[goal] >= num_of_cols
				 || my_map[linearizeCoordinate(x[goal], y[goal])])
			goal++;
		else if (isConnected(linearizeCoordinate(x[start], y[start]),
							 linearizeCoordinate(x[goal], y[goal]))) // cannot find a path from start to goal
		{
			start = goal;
			goal++;
		}
		else
		{
			my_map[obstacle] = false;
			return false;
		}
	}
	return true;
}

bool Instance::isConnected(int start, int goal) const
{
	std::queue<int> open;
	vector<bool> closed(map_size, false);
	open.push(start);
	closed[start] = true;
	while (!open.empty())
	{
		int curr = open.front(); open.pop();
		if (curr == goal)
			return true;
		for (int next : getNeighbors(curr))
		{
			if (closed[next])
				continue;
			open.push(next);
			closed[next] = true;
		}
	}
	return false;
}

void Instance::generateConnectedRandomGrid(int rows, int cols, int obstacles)
{
	cout << "Generate a " << rows << " x " << cols << " grid with " << obstacles << " obstacles. " << endl;
	int i, j;
	num_of_rows = rows + 2;
	num_of_cols = cols + 2;
	map_size = num_of_rows * num_of_cols;
	my_map.resize(map_size, false);
	// Possible moves [WAIT, NORTH, EAST, SOUTH, WEST]
	/*moves_offset[Instance::valid_moves_t::WAIT_MOVE] = 0;
	moves_offset[Instance::valid_moves_t::NORTH] = -num_of_cols;
	moves_offset[Instance::valid_moves_t::EAST] = 1;
	moves_offset[Instance::valid_moves_t::SOUTH] = num_of_cols;
	moves_offset[Instance::valid_moves_t::WEST] = -1;*/

	// add padding
	i = 0;
	for (j = 0; j < num_of_cols; j++)
		my_map[linearizeCoordinate(i, j)] = true;
	i = num_of_rows - 1;
	for (j = 0; j < num_of_cols; j++)
		my_map[linearizeCoordinate(i, j)] = true;
	j = 0;
	for (i = 0; i < num_of_rows; i++)
		my_map[linearizeCoordinate(i, j)] = true;
	j = num_of_cols - 1;
	for (i = 0; i < num_of_rows; i++)
		my_map[linearizeCoordinate(i, j)] = true;

	// add obstacles uniformly at random
	i = 0;
	while (i < obstacles)
	{
		int loc = rand() % map_size;
		if (addObstacle(loc))
		{
			printMap();
			i++;
		}
	}
}

bool Instance::loadMap()
{
	using namespace boost;
	using namespace std;
	
	ifstream myfile(map_fname.c_str());
	if (!myfile.is_open())
		return false;
	
	string line;
	if (!getline(myfile, line))
	{
		setError("Empty map file or read error");
		myfile.close();
		return false;
	}
	
	// Validate PGM header
	if (line.length() >= 2 && line[0] == 'P')
	{
		if (!isValidPGMHeader(line))
		{
			setError("Invalid PGM format: " + line);
			myfile.close();
			return false;
		}
		
		if (line.substr(0, 2) == "P5") // Binary PGM
		{
			if (!parseMapDimensions(myfile, line) || !validateDimensions())
			{
				myfile.close();
				return false;
			}
			
			if (!parsePGMBinary(myfile))
			{
				myfile.close();
				return false;
			}
		}
		else if (line.substr(0, 2) == "P2") // ASCII PGM
		{
			if (!parseMapDimensions(myfile, line) || !validateDimensions())
			{
				myfile.close();
				return false;
			}
			
			if (!parsePGMASCII(myfile))
			{
				myfile.close();
				return false;
			}
		}
		
		myfile.close();
		printMap();
		return true;
	}
	else 
	{
		if (line[0] == 't') // Nathan's benchmark
		{
			char_separator<char> sep(" ");
			getline(myfile, line);
			tokenizer<char_separator<char>> tok(line, sep);
			auto beg = tok.begin();
			if (beg == tok.end())
			{
				setError("Invalid Nathan's benchmark format");
				myfile.close();
				return false;
			}
			beg++;
			if (beg == tok.end())
			{
				setError("Missing number of rows in Nathan's format");
				myfile.close();
				return false;
			}
			num_of_rows = atoi((*beg).c_str());
			
			getline(myfile, line);
			tokenizer<char_separator<char>> tok2(line, sep);
			beg = tok2.begin();
			if (beg == tok2.end())
			{
				setError("Invalid Nathan's benchmark format line 2");
				myfile.close();
				return false;
			}
			beg++;
			if (beg == tok2.end())
			{
				setError("Missing number of cols in Nathan's format");
				myfile.close();
				return false;
			}
			num_of_cols = atoi((*beg).c_str());
			getline(myfile, line); // skip "map"
		}
		else // my benchmark
		{
			char_separator<char> sep(",");
			tokenizer<char_separator<char>> tok(line, sep);
			auto beg = tok.begin();
			if (beg == tok.end())
			{
				setError("Invalid custom benchmark format");
				myfile.close();
				return false;
			}
			num_of_rows = atoi((*beg).c_str());
			beg++;
			if (beg == tok.end())
			{
				setError("Missing number of cols in custom format");
				myfile.close();
				return false;
			}
			num_of_cols = atoi((*beg).c_str());
		}
		
		if (!validateDimensions())
		{
			myfile.close();
			return false;
		}
		
		map_size = num_of_cols * num_of_rows;
		my_map.resize(map_size, false);
		
		// read map (and start/goal locations)
		for (int i = 0; i < num_of_rows; i++)
		{
			if (!getline(myfile, line))
			{
				setError("Insufficient map data at row " + to_string(i));
				myfile.close();
				return false;
			}
			
			if ((int)line.length() < num_of_cols)
			{
				setError("Row " + to_string(i) + " has insufficient columns: " + to_string(line.length()) + " < " + to_string(num_of_cols));
				myfile.close();
				return false;
			}
			
			for (int j = 0; j < num_of_cols; j++)
			{
				my_map[linearizeCoordinate(i, j)] = (line[j] != '.');
			}
		}
		myfile.close();
		printMap();
		return true;
	}
}


void Instance::printMap() const
{
	for (int i = 0; i < num_of_rows; i++)
	{
		for (int j = 0; j < num_of_cols; j++)
		{
			if (this->my_map[linearizeCoordinate(i, j)])
				cout << '@';
			else
				cout << '.';
		}
		cout << endl;
	}
}


void Instance::saveMap() const
{
	ofstream myfile;
	myfile.open(map_fname);
	if (!myfile.is_open())
	{
		cout << "Fail to save the map to " << map_fname << endl;
		return;
	}
	myfile << num_of_rows << "," << num_of_cols << endl;
	for (int i = 0; i < num_of_rows; i++)
	{
		for (int j = 0; j < num_of_cols; j++)
		{
			if (my_map[linearizeCoordinate(i, j)])
				myfile << "@";
			else
				myfile << ".";
		}
		myfile << endl;
	}
	myfile.close();
}


bool Instance::loadAgents()
{
	using namespace std;
	using namespace boost;

	string line;
	ifstream myfile(agent_fname.c_str());
	if (!myfile.is_open())
		return false;

	getline(myfile, line);
	if (line[0] == 'v') // Nathan's benchmark
	{
		if (num_of_agents == 0)
		{
			setError("The number of agents should be larger than 0");
			return false;
		}
		start_locations.resize(num_of_agents);
		goal_locations.resize(num_of_agents);

		vector<int> ids(num_of_agents);
		if (agent_indices != "")
		{
			char_separator<char> sep(",");
			tokenizer< char_separator<char> > chars(agent_indices, sep);
			int i = 0;
			for (auto c : chars)
			{
				ids[i] = atoi(c.c_str());
				if (i > 0 && ids[i] <= ids[i - 1])
				{
					setError("The indices of the agents should be strictly increasing");
					return false;
				}
				i++;
			}
		}
		else
		{
			for (int i = 0; i < num_of_agents; i++)
				ids[i] = i;
		}
		char_separator<char> sep("\t");	
		int count = 0;
		int i = 0;
		while(i < num_of_agents)
		{
			getline(myfile, line);
			if (count == ids[i])
			{
				tokenizer< char_separator<char> > tok(line, sep);
				tokenizer< char_separator<char> >::iterator beg = tok.begin();
				beg++; // skip the first number
				beg++; // skip the map name
				beg++; // skip the columns
				beg++; // skip the rows
					   // read start [row,col] for agent i
				int col = atoi((*beg).c_str());
				beg++;
				int row = atoi((*beg).c_str());
				start_locations[i] = linearizeCoordinate(row, col);
				// read goal [row,col] for agent i
				beg++;
				col = atoi((*beg).c_str());
				beg++;
				row = atoi((*beg).c_str());
				goal_locations[i] = linearizeCoordinate(row, col);
				i++;
			}
			count++;
		}
	}
	else // My benchmark
	{
		char_separator<char> sep(",");
		tokenizer<char_separator<char>> tok(line, sep);
		tokenizer<char_separator<char>>::iterator beg = tok.begin();
		num_of_agents = atoi((*beg).c_str());
		start_locations.resize(num_of_agents);
		goal_locations.resize(num_of_agents);
		for (int i = 0; i < num_of_agents; i++)
		{
			getline(myfile, line);
			tokenizer<char_separator<char>> col_tok(line, sep);
			tokenizer<char_separator<char>>::iterator c_beg = col_tok.begin();
			pair<int, int> curr_pair;
			// read start [row,col] for agent i
			int row = atoi((*c_beg).c_str());
			c_beg++;
			int col = atoi((*c_beg).c_str());
			start_locations[i] = linearizeCoordinate(row, col);
			// read goal [row,col] for agent i
			c_beg++;
			row = atoi((*c_beg).c_str());
			c_beg++;
			col = atoi((*c_beg).c_str());
			goal_locations[i] = linearizeCoordinate(row, col);
		}
	}
	myfile.close();
	return true;

}


void Instance::printAgents() const
{
	for (int i = 0; i < num_of_agents; i++)
	{
		cout << "Agent" << i << " : S=(" << getRowCoordinate(start_locations[i]) << "," << getColCoordinate(start_locations[i])
			 << ") ; G=(" << getRowCoordinate(goal_locations[i]) << "," << getColCoordinate(goal_locations[i]) << ")" << endl;
	}
}


void Instance::saveAgents() const
{
	ofstream myfile;
	myfile.open(agent_fname);
	if (!myfile.is_open())
	{
		cout << "Fail to save the agents to " << agent_fname << endl;
		return;
	}
	myfile << num_of_agents << endl;
	for (int i = 0; i < num_of_agents; i++)
		myfile << getRowCoordinate(start_locations[i]) << "," << getColCoordinate(start_locations[i]) << ","
			<< getRowCoordinate(goal_locations[i]) << "," << getColCoordinate(goal_locations[i]) << "," << endl;
	myfile.close();
}

// Safe PGM parsing helper methods
bool Instance::isValidPGMHeader(const string& magic) const
{
	if (magic.length() < 2 || magic[0] != 'P')
		return false;
	
	return (magic.substr(0, 2) == "P2" || magic.substr(0, 2) == "P5");
}

bool Instance::parseMapDimensions(ifstream& file, string& line)
{
	using namespace boost;
	
	// Skip comments and empty lines to find dimensions
	int lines_read = 0;
	while (getline(file, line) && lines_read < 100) // Prevent infinite loops
	{
		lines_read++;
		
		// Skip comment lines and empty/whitespace-only lines
		if (line.empty() || line[0] == '#' || line.find_first_not_of(" \t\r\n") == string::npos)
			continue;
			
		// Handle inline comments - remove everything after '#' 
		size_t comment_pos = line.find('#');
		if (comment_pos != string::npos)
		{
			line = line.substr(0, comment_pos);
			// Re-check if line is now empty after removing inline comment
			if (line.empty() || line.find_first_not_of(" \t\r\n") == string::npos)
				continue;
		}
			
		char_separator<char> sep(" ");
		tokenizer<char_separator<char>> tok(line, sep);
		auto beg = tok.begin();
		
		if (beg == tok.end())
		{
			setError("Invalid dimension line format");
			return false;
		}
		
		num_of_cols = atoi((*beg).c_str());
		beg++;
		
		if (beg == tok.end())
		{
			setError("Missing number of rows in PGM");
			return false;
		}
		
		num_of_rows = atoi((*beg).c_str());
		return true;
	}
	
	setError("Could not find valid dimensions in PGM file after " + to_string(lines_read) + " lines");
	return false;
}

bool Instance::validateDimensions() const
{
	if (num_of_rows <= 0 || num_of_cols <= 0)
	{
		setError("Invalid map dimensions: " + to_string(num_of_rows) + "x" + to_string(num_of_cols));
		return false;
	}
	
	if (num_of_rows > 10000 || num_of_cols > 10000)
	{
		setError("Map dimensions too large: " + to_string(num_of_rows) + "x" + to_string(num_of_cols));
		return false;
	}
	
	return true;
}

bool Instance::parsePGMBinary(ifstream& file)
{
	using namespace std;
	
	string line;
	// Skip max value line and any empty lines
	int lines_read = 0;
	while (getline(file, line) && lines_read < 10)
	{
		lines_read++;
		if (!line.empty() && line.find_first_not_of(" \t\r\n") != string::npos)
			break;
	}
	
	map_size = num_of_cols * num_of_rows;
	if (map_size <= 0 || map_size > 100000000) // 100M limit
	{
		setError("Invalid map size for binary PGM: " + to_string(map_size));
		return false;
	}
	
	my_map.resize(map_size, false);
	
	// Read binary data with bounds checking
	vector<unsigned char> data(map_size);
	file.read(reinterpret_cast<char*>(data.data()), map_size);
	
	if (file.gcount() != map_size)
	{
		setError("Insufficient binary data: expected " + to_string(map_size) + " bytes, got " + to_string(file.gcount()));
		return false;
	}
	
	for (int i = 0; i < map_size; i++)
	{
		int file_row = i / num_of_cols;
		int col = i % num_of_cols;
		int map_row = num_of_rows - 1 - file_row;
		my_map[linearizeCoordinate(map_row, col)] = (data[i] != 254); // 254 is free space
	}
	
	return true;
}

bool Instance::parsePGMASCII(ifstream& file)
{
	using namespace boost;
	using namespace std;
	
	string line;
	// Skip max value line and any empty lines  
	int lines_read = 0;
	while (getline(file, line) && lines_read < 10)
	{
		lines_read++;
		if (!line.empty() && line.find_first_not_of(" \t\r\n") != string::npos)
			break;
	}
	
	map_size = num_of_cols * num_of_rows;
	if (map_size <= 0 || map_size > 100000000) // 100M limit
	{
		setError("Invalid map size for ASCII PGM: " + to_string(map_size));
		return false;
	}
	
	my_map.resize(map_size, false);
	
	// Read ASCII pixel data with bounds checking
	for (int i = 0; i < num_of_rows; i++)
	{
		// Skip empty lines and get the next line with data
		int empty_lines = 0;
		do {
			if (!getline(file, line))
			{
				setError("Insufficient ASCII data at row " + to_string(i));
				return false;
			}
			empty_lines++;
		} while ((line.empty() || line.find_first_not_of(" \t\r\n") == string::npos) && empty_lines < 10);
		
		if (empty_lines >= 10)
		{
			setError("Too many empty lines at row " + to_string(i));
			return false;
		}
		
		// Handle inline comments - remove everything after '#'
		size_t comment_pos = line.find('#');
		if (comment_pos != string::npos)
		{
			line = line.substr(0, comment_pos);
			// Re-check if line is now empty after removing inline comment
			if (line.empty() || line.find_first_not_of(" \t\r\n") == string::npos)
			{
				setError("Line became empty after removing inline comment at row " + to_string(i));
				return false;
			}
		}
		
		char_separator<char> sep(" ");
		tokenizer<char_separator<char>> tok(line, sep);
		auto beg = tok.begin();
		
		for (int j = 0; j < num_of_cols; j++)
		{
			if (beg == tok.end())
			{
				setError("Insufficient pixel data at row " + to_string(i) + ", col " + to_string(j));
				return false;
			}
			
			int pixel_value = atoi((*beg).c_str());
			// Flip Y: File row 'i' corresponds to Map row 'num_of_rows - 1 - i'
			int map_row = num_of_rows - 1 - i;
			my_map[linearizeCoordinate(map_row, j)] = (pixel_value < 254); // Values >= 254 are free space, < 254 are obstacles
			beg++;
		}
	}
	
	return true;
}

list<int> Instance::getNeighbors(int curr) const
{
	list<int> neighbors;
	int candidates[4] = { curr + 1, curr - 1, curr + num_of_cols, curr - num_of_cols };
	for (int next : candidates)
	{
		if (validMove(curr, next))
			neighbors.emplace_back(next);
	}
	return neighbors;
}