#pragma once
#include <fstream>
class Textfile{
public:
	Textfile(const char* title);
	~Textfile();
	void Close();

	std::ofstream txt;
	int idx = 1;
	double elapsed_time = 0;
	bool valid = true;
private:

};