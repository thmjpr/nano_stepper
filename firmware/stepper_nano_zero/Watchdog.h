#pragma once



class Watchdog
{
private:
	bool enabled = false;
	
public:
	void setup();
	void clear();
};
