/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <clopema_rubbing/Calibrate.h>
#include <clopema_rubbing/Stiffness.h>
#include <clopema_rubbing/Rubbing.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <sstream>

//Global data
FILE *fpSerial = NULL;   //serial port file pointer
char port[20];    //port name
int baud;     //baud rate

//Initialize serial port, return file descriptor
FILE *serialInit(char * port, int baud)
{
	int BAUD = 0;
	int fd = -1;
	struct termios newtio;
	FILE *fp = NULL;

	//Open the serial port as a file descriptor for low level configuration
	// read/write, not controlling terminal for process,
	fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY );
	if ( fd<0 )
	{
		ROS_ERROR("serialInit: Could not open serial device %s",port);
		return fp;
	}

	// set up new settings
	memset(&newtio, 0,sizeof(newtio));
	newtio.c_cflag =  CS8 | CLOCAL | CREAD;  //no parity, 1 stop bit

	newtio.c_iflag = IGNCR;    //ignore CR, other options off
	newtio.c_iflag |= IGNBRK;  //ignore break condition

	newtio.c_oflag = 0;        //all options off

	newtio.c_lflag = ICANON;     //process input as lines

	// activate new settings
	tcflush(fd, TCIFLUSH);
	//Look up appropriate baud rate constant
	switch (baud)
	{
	case 38400:
		BAUD = B38400;
		break;
	case 19200:
		BAUD  = B19200;
		break;
	case 9600:
	default:
		BAUD  = B9600;
		break;
	case 4800:
		BAUD  = B4800;
		break;
	case 2400:
		BAUD  = B2400;
		break;
	case 1800:
		BAUD  = B1800;
		break;
	case 1200:
		BAUD  = B1200;
		break;
	}  //end of switch baud_rate
	if (cfsetispeed(&newtio, BAUD) < 0 || cfsetospeed(&newtio, BAUD) < 0)
	{
		ROS_ERROR("serialInit: Failed to set serial baud rate: %d", baud);
		close(fd);
		return NULL;
	}
	tcsetattr(fd, TCSANOW, &newtio);
	tcflush(fd, TCIOFLUSH);

	//Open file as a standard I/O stream
	fp = fdopen(fd, "r+");
	if (!fp) {
		ROS_ERROR("serialInit: Failed to open serial stream %s", port);
		fp = NULL;
	}
	return fp;
} //serialInit

void flush()
{
	char response[200];
	while (fgets(response, 200, fpSerial) != NULL);
}

int get_response()
{
	char response[200];
	int ret = 0;
	int i = 0;
	ROS_INFO("I'm here");

	while (!ros::isShuttingDown())
	{
		while (fgets(response, 200, fpSerial) == NULL && !ros::isShuttingDown())
		{
			printf("\r%d", i++);
			usleep(10000);
		}
		printf("\n");

		ROS_INFO("Got line: '%s'", response);

		if (sscanf(response, "%d", &ret) == 1)
			break;
	}

	return ret;
}

bool serve_calibrate(clopema_rubbing::Calibrate::Request &req, clopema_rubbing::Calibrate::Response &res)
{
	flush();

	/* send request */
	fprintf(fpSerial, "1 0\n");

	/* wait for reply */
	res.count1 = get_response();
	res.count2 = get_response();

	return true;
}

bool serve_stiffness(clopema_rubbing::Stiffness::Request &req, clopema_rubbing::Stiffness::Response &res)
{
	flush();

	/* send request */
	fprintf(fpSerial, "2 %u\n", req.stiffness);

	/* wait for reply */
	res.count1 = get_response();
	res.count2 = get_response();
	res.stiffness = get_response();

	return true;
}

bool serve_rubbing(clopema_rubbing::Rubbing::Request &req, clopema_rubbing::Rubbing::Response &res)
{
	flush();

	/* send request */
	fprintf(fpSerial, "3 %u\n", req.times);

	/* wait for reply */
	res.count1 = get_response();
	res.count2 = get_response();

	return true;
}

void print_help(const char *prog)
{
	std::cout << "Usage: " << prog << " [--help] [--limb L] [--port p] [--baud b]" << std::endl;
	std::cout << std::endl;
	std::cout << "    --limb L        The limb this motor server controls" << std::endl;
	std::cout << "                    Give 1 for the right limb and 2 for the left one" << std::endl;
	std::cout << "    --port p        The serial port for communication (default /dev/ttyUSB0 or ...1 for right/left accordingly)" << std::endl;
	std::cout << "    --baud b        The baud rate of serial port (default 9600)" << std::endl;
}

int parse_args(int argc, char **argv)
{
	int limb = -1;
	bool got_limb = false;
	int i;
	for (i = 1; i < argc; ++i)
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0)
			break;
		else if (strcmp(argv[i], "-l") == 0 || strcmp(argv[i], "--limb") == 0)
		{
			if (i + 1 < argc)
			{
				++i;
				if (sscanf(argv[i], "%d", &limb) != 1 || (limb != 1 && limb != 2))
				{
					std::cout << "invalid limb" << std::endl << std::endl;
					break;
				}
				else
					got_limb = true;
			}
			else
				break;
		}
		else if (strcmp(argv[i], "-p") == 0 || strcmp(argv[i], "--port") == 0)
		{
			if (i + 1 < argc)
			{
				++i;
				strncpy(port, argv[i], sizeof port);
				port[sizeof port - 1] = '\0';
			}
			else
				break;
		}
		else if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0)
		{
			if (i + 1 < argc)
			{
				++i;
				if (sscanf(argv[i], "%d", &baud) != 1)
				{
					std::cout << "invalid baud rate" << std::endl << std::endl;
					break;
				}
			}
			else
				break;
		}
	if (!got_limb || i < argc)
	{
		print_help(argv[0]);
		return -1;
	}

	return limb;
}

int main(int argc, char **argv)
{
	int limb;
	int err;

	port[0] = '\0';
	baud = 9600;

	limb = parse_args(argc, argv);
	if (limb < 1)
		return 0;
	if (port[0] == '\0')
	{
		if (limb == 1)
			strcpy(port, "/dev/ttyUSB0");
		else
			strcpy(port, "/dev/ttyUSB1");
	}

	std::string name;
	std::ostringstream sout;
	sout << "r" << limb << "_gripper";

	name = sout.str();

	//Initialize ROS
	ros::init(argc, argv, name + "_rubbing");
	ros::NodeHandle rosNode;
	ROS_INFO("Rubbing starting");

	//Open and initialize the serial port to the uController
	ROS_INFO("connection initializing (%s) at %d baud", port, baud);
	fpSerial = serialInit(port, baud);
	if (!fpSerial )
	{
		ROS_ERROR("unable to create a new serial port");
		return 1;
	}

	ROS_INFO("serial connection successful");

	ros::ServiceServer calibrate_service = rosNode.advertiseService(name + "/CalibrateRubbing", serve_calibrate);
	ros::ServiceServer stiffness_service = rosNode.advertiseService(name + "/SetRubbingStiffness", serve_stiffness);
	ros::ServiceServer rubbing_service = rosNode.advertiseService(name + "/Rub", serve_rubbing);

	//Process ROS messages and send serial commands to uController
	ros::spin();

	fclose(fpSerial);
	ROS_INFO("Rubbing stopping");
	return 0;
}
