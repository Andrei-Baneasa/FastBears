#include <opencv2/opencv.hpp>
#include <raspicam_cv.h>
#include <iostream>
#include <chrono>
#include <ctime>
#include <stdio.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

using namespace std;
using namespace cv;
using namespace raspicam;

Mat roi,frame,frameGray,frameThresh,frameEdge,frameFinal,frameFinalDuplicate;
Mat ROILane, STOP, histo, frame_stop, frameFinalStop,histo2;
Mat PARK, histop, frame_park, frameFinalPark, histo2p;
Mat PRIORITY, histopr, frame_priority, frameFinalPriority, histo2pr;
RaspiCam_Cv Camera;
vector<int> histrogramPark;
vector<int> histrogramStop;
vector<int> histrogramPriority;
int LanePos, Result=0,frameCenter,serial_port,i,k=0, l=0, j=0;
stringstream ss;

 void Setup ( int argc,char **argv, RaspiCam_Cv &Camera )
 {
    Camera.set ( CAP_PROP_FRAME_WIDTH,  ( "-w",argc,argv,640 ) );
    Camera.set ( CAP_PROP_FRAME_HEIGHT,  ( "-h",argc,argv,480 ) );
    Camera.set ( CAP_PROP_BRIGHTNESS, ( "-br",argc,argv,60 ) );           //Camera setup 
    Camera.set ( CAP_PROP_CONTRAST ,( "-co",argc,argv,50 ) );
    Camera.set ( CAP_PROP_SATURATION,  ( "-sa",argc,argv,50 ) );
    Camera.set ( CAP_PROP_GAIN,  ( "-g",argc,argv ,50 ) );
    Camera.set ( CAP_PROP_FPS,  ( "-fps",argc,argv,60));
}

void Capture()
{
	Camera.grab();
	Camera.retrieve(frame);                                          // Getting Video from the camera
	histo = frame(Rect(0,0,640,480));
	histop= frame(Rect(0,0,640,480));
	histopr= frame(Rect(0,0,640,480));
	
}

void Stop_detection()
{
    cvtColor(histo, histo2, COLOR_BGR2HSV);
    inRange(histo2, Scalar(160, 100, 100), Scalar(180, 255, 255), frame_stop);
    histrogramStop.resize(640);
    histrogramStop.clear();
    cvtColor(frame_stop, frameFinalStop, COLOR_RGB2BGR);
    
    for(int i=0; i<640; i++)       //frame.size().width
    {
	STOP = frameFinalStop(Rect(i,0,1,480));
	divide(255, STOP, STOP);
	histrogramStop.push_back((int)(sum(STOP)[0])); 
	if(histrogramStop[i]>0)
	{
	    k++;
	    }
    }
    
   if(k>15)
   {
		
       serialPutchar(serial_port, 's');
       }
    
    
    
}

void Park_detection()
{
    cvtColor(histop, histo2p, COLOR_BGR2HSV);
    inRange(histo2p, Scalar(110, 110, 100), Scalar(130, 255, 255), frame_park);
    histrogramPark.resize(640);
    histrogramPark.clear();
    cvtColor(frame_park, frameFinalPark, COLOR_RGB2BGR);
    
    for(int i=0; i<640; i++)       //frame.size().width
    {
	PARK = frameFinalPark(Rect(i,0,1,480));
	divide(255, PARK, PARK);
	histrogramPark.push_back((int)(sum(PARK)[0])); 
	if(histrogramPark[i]>0)
	{
	    l++;
	    }
    }
    
   if(l>15)
   {
		
       serialPutchar(serial_port, 'p');
       }
    
    
    
}

void Priority_detection()
{
    cvtColor(histopr, histo2pr, COLOR_BGR2HSV);
    inRange(histo2pr, Scalar(0, 100, 215), Scalar(180, 255, 255), frame_priority);
    histrogramPriority.resize(640);
    histrogramPriority.clear();
    cvtColor(frame_priority, frameFinalPriority, COLOR_RGB2BGR);
    
    for(int i=0; i<640; i++)       //frame.size().width
    {
	PRIORITY = frameFinalPriority(Rect(i,0,1,480));
	divide(255, PRIORITY, PRIORITY);
	histrogramPriority.push_back((int)(sum(PRIORITY)[0])); 
	if(histrogramPriority[i]>0)
	{
	    j++;
	    }
    }
    
   if(j>15)
   {
		
       serialPutchar(serial_port, 'r');
       }
    
    
    
}


int main(int argc,char **argv)
{
	
	Setup(argc, argv, Camera);
	cout<<"Connecting to camera"<<endl;
	if (!Camera.open())
	{
		
	cout<<"Failed to Connect"<<endl;
     }
     
     cout<<"Camera Id = "<<Camera.getId()<<endl;
     
      if ((serial_port = serialOpen ("/dev/ttyS0",57600)) < 0)	/* open serial port */
    {
	fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
	return 1 ;
    }	

    if (wiringPiSetup () == -1)					/* initializes wiringPi setup */
    {
	fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
	return 1 ;
    }

    while(1)
    {
	auto start = std::chrono::system_clock::now();

    Capture();
    Stop_detection();
    Park_detection();
    Priority_detection();
    
    
    
    namedWindow("procesare", WINDOW_KEEPRATIO);
    //moveWindow("procesare", 0, 100);
    resizeWindow("procesare", 640, 480);
    imshow("procesare", frame);
    
    namedWindow("stop", WINDOW_KEEPRATIO);
    //moveWindow("stop", 640, 100);
    resizeWindow("stop", 640, 480);
    imshow("stop", frame_stop);
    
    namedWindow("park", WINDOW_KEEPRATIO);
    //moveWindow("park", 640, 400);
    resizeWindow("park", 640, 480);
    imshow("park", frame_park);
    
    /*namedWindow("priority", WINDOW_KEEPRATIO);
    //moveWindow("priority", 640, 400);
    resizeWindow("priority", 640, 480);
    imshow("priority", frame_priority);*/
    
    waitKey(1);
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;
    
    float t = elapsed_seconds.count();
    int FPS = 1/t;
    cout<<"FPS = "<<FPS<<" "<<"k= "<<k<<" "<<"l="<<l<<" "<<"j="<<j<<endl;
    

    k=0;
    l=0;
    j=0;
    
    }

    
    return 0;
     
}
