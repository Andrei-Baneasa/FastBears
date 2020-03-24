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
Mat ROILane;
RaspiCam_Cv Camera;
vector<int> histrogramLane;
int LanePos, Result=0,frameCenter,serial_port,i;
stringstream ss;
int lastVal=0;

//Machine Learning variables
CascadeClassifier Stop_Cascade;
Mat frame_stop RoI_Stop, gray_stop;
vector<Rect>Stop;

 void Setup ( int argc,char **argv, RaspiCam_Cv &Camera )
 {
    Camera.set ( CAP_PROP_FRAME_WIDTH,  ( "-w",argc,argv,640 ) );
    Camera.set ( CAP_PROP_FRAME_HEIGHT,  ( "-h",argc,argv,480 ) );
    Camera.set ( CAP_PROP_BRIGHTNESS, ( "-br",argc,argv,60 ) );
    Camera.set ( CAP_PROP_CONTRAST ,( "-co",argc,argv,50 ) );
    Camera.set ( CAP_PROP_SATURATION,  ( "-sa",argc,argv,50 ) );
    Camera.set ( CAP_PROP_GAIN,  ( "-g",argc,argv ,50 ) );
    Camera.set ( CAP_PROP_FPS,  ( "-fps",argc,argv,60));
}

void Capture()
{
	Camera.grab();
	Camera.retrieve(frame);
	roi = frame(Rect(0,440,640,10));
    //cvtColor(frame, frame, COLOR_BGR2RGB);
	cvtColor(frame, frame_Stop, COLOR_BGR2RGB);
}

void Threshold()
{
	cvtColor(roi, frameGray, COLOR_RGB2GRAY);
	inRange(frameGray, 220, 255, frameThresh);
	Canny(frameThresh,frameEdge, 100,200 , 3, false);//frameGray
	add(frameThresh, frameEdge, frameFinal);
	cvtColor(frameFinal, frameFinal, COLOR_GRAY2RGB);
	cvtColor(frameFinal, frameFinalDuplicate, COLOR_RGB2BGR);   //used in histrogram function only
}

void Histrogram()
{
    histrogramLane.resize(640);
    histrogramLane.clear();

    for(int i=0; i<640; i++)       //frame.size().width
    {
	ROILane = frameFinalDuplicate(Rect(i,0,1,10));
	divide(255, ROILane, ROILane);
	histrogramLane.push_back((int)(sum(ROILane)[0]));
    }
}
void LaneFinder()
{
    vector<int>:: iterator LeftPtr;
    LeftPtr = max_element(histrogramLane.begin(), histrogramLane.end());
    LanePos = distance(histrogramLane.begin(), LeftPtr);

    //vector<int>:: iterator RightPtr;
    //RightPtr = max_element(histrogramLane.begin() +220, histrogramLane.end());
    //RightLanePos = distance(histrogramLane.begin(), RightPtr);

    line(frameFinal, Point2f(LanePos, 0), Point2f(LanePos, 100), Scalar(0, 255,0), 3);
    //line(frameFinal, Point2f(RightLanePos, 0), Point2f(RightLanePos, 240), Scalar(0,255,0), 2);


}
/*void Stop_detection()
{
    if(!Stop_Cascade.load(""))//path for the machine learning of the stop sign
    {
        cout<<("Cant open the file")
    }
	RoI_Stop=frame_Stop(0,0 400, 240));
	cvtColor(RoI_Stop, gray_stop, COLOR_RGB2GRAY);
	equalizeHist(gray_stop, gray_stop);
	Stop_Cascade.detectMultiScale(gray_stop, Stop);

i	for(int i=0; i<Stop.size(), i++)
	{
		Point p1(Stop[i].x, Stop[i].y);
		Point p2(Stop[i].+Stop[i].width, Stop[i].x+Stop[i].height);

		rectangle(RoI_Stop, p1, p2, Scalar(0, 0, 255), 2);
		putText(RoI_Stop, "Stop Sign", p1, Scalar(0, 0, 255), 2);
	}

}*/


void uartTx(char a, int res)
{
	if(res == (-570))
	{
	    res = 140;
	    for(int l=1; l<=5; l++)
	    {
		res+=l;
		serialPutchar(serial_port, res);


	    }
	    }

	int aux = res;
	serialPutchar(serial_port,a);
   	if(res<0)
	{
	    aux=-aux;
	    serialPutchar(serial_port,'-');
	}

	int aux2 = 0;

	while(aux!=0)
	{
	    int rest=aux%10;
	    aux2=aux2*10+rest;
	    aux=aux/10;
	}
	while(aux2!=0)
	{
	     int cifra= (aux2%10)+48;
	     serialPutchar(serial_port,char(cifra));
	     aux2=aux2/10;
	}

	serialPutchar(serial_port,';');


}

void LaneCenter()
{
    //laneCenter = (RightLanePos-LeftLanePos)/2 +LeftLanePos;
    frameCenter = 570;

    //line(frameFinal, Point2f(laneCenter,0), Point2f(laneCenter,240), Scalar(0,255,0), 3);
    line(frameFinal, Point2f(frameCenter,0), Point2f(frameCenter,100), Scalar(255,0,0), 3);

    Result = LanePos - frameCenter;
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
    Threshold();
    Histrogram();
    LaneFinder();
    LaneCenter();
	//Stop_detection();



    char z='s';
    uartTx(z,Result);

    ss.str(" ");
    ss.clear();
    ss<<"ERR = "<<Result;
    putText(frame, ss.str(), Point2f(1,25), 0,1, Scalar(0,0,255), 1);



    namedWindow("procesare", WINDOW_KEEPRATIO);
    moveWindow("procesare", 0, 100);
    resizeWindow("procesare", 640, 40);
    imshow("procesare", frameFinal);

    namedWindow("original", WINDOW_KEEPRATIO);
    moveWindow("original", 640, 100);
    resizeWindow("original", 640, 480);
    imshow("original", frame);

	/*namedWindow("Stop Sign", WINDOW_KEEPRATIO);
    moveWindow("Stop Sign", 640, 580);
    resizeWindow("Stop Sign", 640, 480);
    imshow("Stop Sign", RoI_Stop);*/

    /*0namedWindow("Final", WINDOW_KEEPRATIO);
    moveWindow("Final", 640, 100);
    resizeWindow("Final", 640, 480);
    imshow("Final", frameFinal);*/


    waitKey(1);
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;

    float t = elapsed_seconds.count();
    int FPS = 1/t;
    cout<<"FPS = "<<FPS<<endl;


    }


    return 0;

}
