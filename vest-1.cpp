/*
This is the first prototype of the vest built by researchers from Universidade Presbiteriana Mackenzie (Brazil)
This program captures a scene and detect a face.
A signal is send to a serial port where the VEST prototype is connected.
The VEST-1 uses a 3x3 motors array located at the back of the vest using 2 Arduinos.
The pattern of the signa is a string starting with "<" and ending with ">"
Considering "�" = FF = 255, to activate all the motors the signal that should be send is "<���������>"
*/

#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <iostream>
#include <stdio.h>
#include <fcntl.h>  /* File Control Definitions          */
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <unistd.h> /* UNIX Standard Definitions         */
#include <errno.h>  /* ERROR Number Definitions          */

using namespace std;
using namespace cv;

void detectAndDraw(Mat& img, CascadeClassifier& cascade,
	CascadeClassifier& nestedCascade,
	double scale, bool tryflip);

bool writeDataToSerial(string port, char data[], size_t size);

string cascadeName;
string nestedCascadeName;

int main(int argc, const char** argv)
{
	VideoCapture capture;
	Mat frame, image;
	string inputName;
	bool tryflip;
	CascadeClassifier cascade, nestedCascade;
	double scale = 1;
	cascadeName = "haarcascade_frontalface_alt.xml";
	tryflip = false;
	inputName = "0"; // Camera 0

	if (!cascade.load(cascadeName))
	{
		cerr << "ERROR: Could not load classifier cascade" << endl;
		return -1;
	}

	if (inputName.empty() || (isdigit(inputName[0]) && inputName.size() == 1))
	{
		int camera = inputName.empty() ? 0 : inputName[0] - '0';
		if (!capture.open(camera))
			cout << "Capture from camera #" << camera << " didn't work" << endl;
	}
	else
	{
		cout << "Could not capture video ..." << endl;
		return -1;
	}

	if (capture.isOpened())
	{
		cout << "Video capturing has been started ..." << endl;
		for (;;)
		{
			capture >> frame;
			if (frame.empty())
				break;
			Mat frame1 = frame.clone();
			detectAndDraw(frame1, cascade, nestedCascade, scale, tryflip);
			char c = (char)waitKey(10);
			if (c == 27 || c == 'q' || c == 'Q')
				break;
		}
	}

	return 0;
}

bool writeDataToSerial(string port, char data[], size_t size) {
	int fd;
	fd = open(port.c_str(), O_RDWR | O_NOCTTY);
	if(fd == 1)
		printf("\n  Error! in Opening %s\n", port.c_str());
	else
		printf("\n %s Opened Successfully\n", port.c_str());

	struct termios SerialPortSettings;	/* Create the structure                          */

	tcgetattr(fd, &SerialPortSettings);	/* Get the current attributes of the Serial port */

	cfsetispeed(&SerialPortSettings,B9600); /* Set Read  Speed as 9600                       */
	cfsetospeed(&SerialPortSettings,B9600); /* Set Write Speed as 9600                       */

	SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
	SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
	SerialPortSettings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
	SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */

	SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
	SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */ 
	
	
	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
	SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */

	SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/

	if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
		printf("\n  ERROR ! in Setting attributes");
	else
		printf("\n  BaudRate = 9600 \n  StopBits = 1 \n  Parity   = none");
		
	/*------------------------------- Write data to serial port -----------------------------*/
	//char write_buffer[] = "<ÿÿÿÿÿÿÿÿÿÿ>";	/* Buffer containing characters to write into port	     */	
	//char write_buffer[] = "<>";
	int  bytes_written  = 0;  	/* Value for storing the number of bytes written to the port */ 

	bytes_written = write(fd, data, size);
	printf("\n  %s written to %s", data, port.c_str());
	printf("\n  %d Bytes written to %s", bytes_written, port.c_str());
	printf("\n +----------------------------------+\n\n");

	close(fd);/* Close the Serial port */

	//return Status;
	return true;
}

void detectAndDraw(Mat& img, CascadeClassifier& cascade,
	CascadeClassifier& nestedCascade,
	double scale, bool tryflip)
{
	double t = 0;
	vector<Rect> faces, faces2;
	const static Scalar colors[] =
	{
		Scalar(255,0,0),
		Scalar(255,128,0),
		Scalar(255,255,0),
		Scalar(0,255,0),
		Scalar(0,128,255),
		Scalar(0,255,255),
		Scalar(0,0,255),
		Scalar(255,0,255)
	};
	Mat gray, smallImg;
	cvtColor(img, gray, COLOR_BGR2GRAY);
	double fx = 1 / scale;
	resize(gray, smallImg, Size(), fx, fx, INTER_LINEAR);
	equalizeHist(smallImg, smallImg);
	t = (double)getTickCount();
	cascade.detectMultiScale(smallImg, faces,
		1.1, 2, 0
		//|CASCADE_FIND_BIGGEST_OBJECT
		//|CASCADE_DO_ROUGH_SEARCH
		| CASCADE_SCALE_IMAGE,
		Size(30, 30));

	/* **** HANDLE SERIAL COMMUNICATION HERE FROM NUMBER OF DETECTED FACES ***** */

	char hasFace[13] = { '<',
		char(255), char(255), char(255), char(255), char(255),
		char(255), char(255), char(255), char(255), char(255),
		'>', '\0' };

	char noFace[13] = { '<',
		char(1), char(1), char(1), char(1), char(1),
		char(1), char(1), char(1), char(1), char(1),
		'>', '\0' };

	if (faces.size() > 0) {
		printf("port: %s", "/dev/ttyACM0");
		writeDataToSerial("/dev/ttyACM0", hasFace, sizeof(hasFace));
	}
	else {
		writeDataToSerial("/dev/ttyACM0", noFace, sizeof(noFace));
	}

	t = (double)getTickCount() - t;
	printf("detection time = %g ms\n", t * 1000 / getTickFrequency());
	for (size_t i = 0; i < faces.size(); i++)
	{
		Rect r = faces[i];
		Mat smallImgROI;
		vector<Rect> nestedObjects;
		Point center;
		Scalar color = colors[i % 8];
		int radius;
		double aspect_ratio = (double)r.width / r.height;
		if (0.75 < aspect_ratio && aspect_ratio < 1.3)
		{
			center.x = cvRound((r.x + r.width*0.5)*scale);
			center.y = cvRound((r.y + r.height*0.5)*scale);
			radius = cvRound((r.width + r.height)*0.25*scale);
			circle(img, center, radius, color, 3, 8, 0);
		}
		else
			rectangle(img, cvPoint(cvRound(r.x*scale), cvRound(r.y*scale)),
				cvPoint(cvRound((r.x + r.width - 1)*scale), cvRound((r.y + r.height - 1)*scale)),
				color, 3, 8, 0);
		if (nestedCascade.empty())
			continue;
		smallImgROI = smallImg(r);
		nestedCascade.detectMultiScale(smallImgROI, nestedObjects,
			1.1, 2, 0
			//|CASCADE_FIND_BIGGEST_OBJECT
			//|CASCADE_DO_ROUGH_SEARCH
			//|CASCADE_DO_CANNY_PRUNING
			| CASCADE_SCALE_IMAGE,
			Size(30, 30));
		for (size_t j = 0; j < nestedObjects.size(); j++)
		{
			Rect nr = nestedObjects[j];
			center.x = cvRound((r.x + nr.x + nr.width*0.5)*scale);
			center.y = cvRound((r.y + nr.y + nr.height*0.5)*scale);
			radius = cvRound((nr.width + nr.height)*0.25*scale);
			circle(img, center, radius, color, 3, 8, 0);
		}
	}
	imshow("result", img);
}