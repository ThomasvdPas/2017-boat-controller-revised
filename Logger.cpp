#include "../MODSERIAL/MODSERIAL.h"
#include "mbed.h"
#include "Logger.h"
#include "utilities/utilities.h"
#include "rtos.h"

bool LogCreated = false;
int logcount = 0;
clock_t t0;
 extern Serial pc;
extern Mutex pcMutex;
//typedef std::chrono::high_resolution_clock Clock;


//This function creates the log file and starts the clocktime.
void initLogger(){
	FILE *file;

	file = fopen(LOG_FILE_LOCATION, "w");
	if (file==NULL){
		LogCreated = false;
	}
	else{
		fputs("TU Delft Solar Boat Logfile", file);
		fclose(file);
	}
	logcount += 1;
	t0 = clock();


	//return t0;

}

/*
 * The logger takes a string message and writes it to log.txt on the mbed. It includes time and a logcounter.
 * Data output is in seconds
 */
void Logger(const char *message){
	FILE *file;
	clock_t t = clock();
	float dt = (((float)t-(float)t0)/CLOCKS_PER_SEC);
	char log_message[LOG_MESSAGE_SIZE];
	char lol[100];
	clock_t t2 = clock();
	snprintf(log_message, LOG_MESSAGE_SIZE, "%f sec \t #%i \t %s \n", dt, logcount, message);
	clock_t t3 = clock();
	snprintf(lol, LOG_MESSAGE_SIZE, "t2 = %lu, t3 = %lu clocks = %ld", t2, t3, CLOCKS_PER_SEC);
	if (!LogCreated) {
		file = fopen("/local/log.txt", "w");
		LogCreated = true;
	}
	else{
		file = fopen("/local/log.txt", "a");
	}
	if (file == NULL) {
		if (LogCreated){
			LogCreated = false;
		return;
		}
	}
	else{
		fputs(log_message,  file);
		fclose(file);
	}

	if (file){
		fclose(file);
	}
	logcount += 1;
}

//const char Log_message(const char*message)
