/*
 * Logger.h
 *
 *  Created on: Feb 11, 2016
 *      Author: Bart
 */

extern bool LogCreated;

#define LOG_FILE_LOCATION "/local/log.txt"
#define LOG_MESSAGE_SIZE	64

void initLogger();
void Logger(const char *message);
