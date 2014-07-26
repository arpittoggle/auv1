#include<cstdio>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "serial_talk.h"
#include <iostream>
#include<cstdlib>
#include <string.h>
#include"cv.h"
#include <stdio.h>
#include<fstream>
#include<time.h>
#include<math.h>
using namespace cv;
using namespace std;
clock_t begin, end;         // time calculation
float time_spent;
float time_interval=0.7;
int my_finish_flag = 0;
int my_start_flag = 0;
ifstream Arduino_Input;
ofstream Arduino_Output;
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;
int n;

// pid variables
FILE *fp;
float countx=time_interval;
int errorx,error_sum=0,prev_error,setpoint=0,pid_output;
float kp=1.0,kd=1.0,ki=1.0,aki=0,sampletime=0.2;
bool ex=true;
// pid function
void cal_pid();

//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20*20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
//names that will appear at the top of each window
const string windowName = "Original Image";
const string windowName1 = "HSV Image";
const string windowName2 = "Thresholded Image";
const string windowName3 = "After Morphological Operations";
const string trackbarWindowName = "Trackbars";
void on_trackbar( int, void* )
{//This function gets called whenever a
	// trackbar position is changed





}

//comment the portion between lines 49 to 68 if dont want trackbars and fix the values in the  scalar (,,,) in the inRange ()in the main routine

void createTrackbars(){
	//create window for trackbars


    namedWindow(trackbarWindowName,0);

	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
    createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar );
    createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar );
    createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar );
    createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar );
    createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar );
    createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar );


}

int main(int argc, char** argv)
{
//port and baud rate initialization
	//port_initialize("/dev/ttyACM0","9600");
	char c1[3];int j; //char c1[] will store the value of angle along with the direction as "+45" , n will store angle of the mid line also j is the loop variable.
	fp=fopen("ipdata.txt","a");
	Mat dst, cdst;	//to be used in hough transform code portion
	Mat cameraFeed,camerafeed1;//these will store the processed IplImage values in the Mat form

	char command[]={'1','2','3'};// array to store the motion up or down
	
	Mat HSV ;//matrix to hold hsv image 
	IplImage *r,*b,*g;//pointers to hold split image 
	begin=clock();
	Mat threshold;
	IplImage *camFed,*camfed1;
		int x=0, y=0;
	int diff=0;int k=0;//k=loop variable and diff contains the difference between two lines detected
	
	//createTrackbars();//comment this to get rid of trackbars
	
	CvCapture* capture;//copture pointer
	
	capture=cvCreateFileCapture(argv[1]);//change this to if you want to capture from file or any other camera 
	
	CvSize size = cvSize(
                     (int)cvGetCaptureProperty( capture,
                                               CV_CAP_PROP_FRAME_WIDTH),

                     (int)cvGetCaptureProperty( capture,
                                               CV_CAP_PROP_FRAME_HEIGHT)
                     );//this is to get size of the capture so that we cam write the frame 

	VideoWriter putld("outputld.avi", CV_FOURCC('X','V','I','D'), 30, size);//these are to create video writter and of the codec of avi form
	VideoWriter puthsv("outputhsv.avi", CV_FOURCC('X','V','I','D'), 30, size);
	VideoWriter putnorm("outputnorm.avi", CV_FOURCC('X','V','I','D'), 30, size);
	VideoWriter putth("outputth.avi", CV_FOURCC('X','V','I','D'), 30, size);

	while(1)
	{

	     camFed=cvQueryFrame(capture);
	     camfed1=cvCreateImage(cvGetSize(camFed),IPL_DEPTH_8U,3);

		 r=cvCreateImage(cvGetSize(camFed),IPL_DEPTH_8U,1);
         g=cvCreateImage(cvGetSize(camFed),IPL_DEPTH_8U,1); 
         b=cvCreateImage(cvGetSize(camFed),IPL_DEPTH_8U,1); 
		 cvSplit(camFed,b,g,r,NULL);//splitting
    	 cvNormalize(g,g,0,255,32);//normalize green
 		 cvNormalize(r,r,0,255,32);//normalize red
 		 cvNormalize(b,b,0,255,32);//normalize blue
 		 cvMerge(b,g,r,NULL,camfed1);//merging
 
 		 cvSmooth(camfed1,camfed1,CV_GAUSSIAN,3,3);//smoothing
 
         Mat K(camfed1);//conversion of iplimage* to mat 
		 camerafeed1=Mat(camfed1);
		 cvtColor(camerafeed1,HSV,COLOR_BGR2HSV);//color conversion

		 inRange(HSV,Scalar(40,0,0),Scalar(256,256,256),threshold);//color detection

		 Canny(threshold, dst, 50, 200,3);//edge detection
 		 cvtColor(dst, cdst, CV_GRAY2BGR);//color conversion
 		 vector<Vec2f>lines;//vector to store informations of detected lines
 
  		 HoughLines(dst, lines, 3,5*CV_PI/180, 100, 0, 0 );//application of hough line function

  		 for( size_t i = 0; i <lines.size(); i++ )
  		 {

   				Point pt1, pt2,ptmid1,ptmid2;//points to hold point one and point two to form a line and ptmid is to hold the same for the mid line 

				float rho,theta;
     			if(i==0 || i==1)//putting constraints for the detection of two lines
     			{ 
     				rho = lines[i][0],theta= lines[i][1];
     
     				double a = cos(theta), b = sin(theta);
     				double x0 = a*rho, y0 = b*rho;
     				pt1.x = cvRound(x0 + 1000*(-b));
     				pt1.y = cvRound(y0 + 1000*(a));
     				pt2.x = cvRound(x0 - 1000*(-b));
     				pt2.y = cvRound(y0 - 1000*(a));
     				line( cdst, pt1, pt2, Scalar(0,255,0), 3, CV_AA);
		

				}

				if(i==1)//when we have two lines' information then....
				{
					diff=lines[1][0]-lines[0][0];//get the difference  in pixel
					lines[i+1][0]=(lines[i-1][0]+lines[i][0])/2;//extending the vector by one and storing the values for the mid line
					lines[i+1][1]=lines[i][1];
					ptmid1.x=cvRound(lines[i+1][0]*cos(lines[i+1][1])+1000*(-sin(lines[i+1][1])));
					ptmid1.y=cvRound(lines[i+1][0]*sin(lines[i+1][1])+1000*(cos(lines[i+1][1])));
					ptmid2.x=cvRound(lines[i+1][0]*cos(lines[i+1][1])-1000*(-sin(lines[i+1][1])));
					ptmid2.y=cvRound(lines[i+1][0]*sin(lines[i+1][1])-1000*(cos(lines[i+1][1])));
					line(cdst,ptmid1,ptmid2 ,Scalar(0,255,255),3,CV_AA);
					if(lines[i+1][1]<1.57 && lines[i+1][1]>.17444)//turnig right condition
				    {
		
						c1[0]='+';//
						n=(int)(lines[i+1][1]*180/CV_PI);
						end=clock();
		time_spent = (float)(end - begin) / CLOCKS_PER_SEC;
		if(time_spent>time_interval)
		//create a char array and store the ASCII value in that
		{ /*char angl[3];
		if(angleint<0)
		angl[0]='-';
		else
		angl[0]='+';
		angl[1]='0'+angleint/10;
		angl[2]='0'+angleint-10*(angleint/10);					//C
		//com->sendArray(angl,2);
		
		//int temp=10;
		Arduino_Output << (char *)angl << std::flush;*/
		cal_pid();
		char outangle[3];
 		if(outangle<0)
		outangle[0]='-';
		else
		outangle[0]='+';
		outangle[1]='0'+pid_output/10;
		outangle[2]='0'+pid_output-10*(pid_output/10);
		//Arduino_Output << (char *)outangle << std::flush;
		//cvWaitKey(900);					//C
		begin=clock();
		fprintf(fp,"%.3f %d %d\n",countx,n,pid_output);
		printf("%.3f %d %d\n",countx,n,pid_output);
		countx+=time_interval;
		}
		
						/*c1[2]=n%10+48;//storing the value of angle in degrees into the c[]by extracting the digits
						n=n/10;
						c1[1]=n%10+48;
						for(j=0;j<3;j++)
						{
							send_via_port((char *)(c1+j),"char",1);//sending to the serial monitor of arduino
						}*/
					    std::cout  /*<< ptmid1.x << " " << ptmid1.y << " " << ptmid2.x << " " << ptmid2.y << " " << lines[i+1][0]*/ << " " << 'D' <<" "<<(int)(lines[i+1][1]*180/CV_PI);
					}
					else if(lines[i+1][1]>1.57 && lines[i+1][1]<2.96556)//condition to turn left
					{
		
						//c1[0]='-';
						n=(int)(180-lines[i+1][1]*180/CV_PI);
						n=-1*n;
						end=clock();
		time_spent = (float)(end - begin) / CLOCKS_PER_SEC;
		if(time_spent>time_interval)
		//create a char array and store the ASCII value in that
		{ /*char angl[3];
		if(angleint<0)
		angl[0]='-';
		else
		angl[0]='+';
		angl[1]='0'+angleint/10;
		angl[2]='0'+angleint-10*(angleint/10);					//C
		//com->sendArray(angl,2);
		
		//int temp=10;
		Arduino_Output << (char *)angl << std::flush;*/
		cal_pid();
		char outangle[3];
 		if(outangle<0)
		outangle[0]='-';
		else
		outangle[0]='+';
		outangle[1]='0'+pid_output/10;
		outangle[2]='0'+pid_output-10*(pid_output/10);
		//Arduino_Output << (char *)outangle << std::flush;
		//cvWaitKey(900);					//C
		begin=clock();
		fprintf(fp,"%.3f %d %d\n",countx,n,pid_output);
		printf("%.3f %d %d\n",countx,n,pid_output);
		countx+=time_interval;
		}
		
						/*c1[2]=n%10+48;
						n=n/10;
						c1[1]=n%10+48;
						for(j=0;j<3;j++)
						{
							send_via_port((char *)(c1+j),"char",1);
						}*/
						std::cout /*<< ptmid1.x << " " << ptmid1.y << " " << ptmid2.x << " " << ptmid2.y << " " << lines[i+1][0] */<< " " << 'A'<<" "<<(int)(180-lines[i+1][1]*180/CV_PI);
					}
					else//no turn condition
					std::cout << "X";	


     				std::cout << '\n';

					if(diff<0)
					diff=(-1)*diff;
					if(k==3)
					{
						k=0;
 
					}
					if(diff>200)
					{
						command[k]='U';k++;
					}
					else if(diff>100&&diff<180)
					{
						command[k]='D';
						k++;
					}
	
					if(command[0]==command[1]&&command[1]==command[2])
					std::cout << command[1]<<'\n';
	
				}


				if(i>1)break;
  			}


 //this is the the portion of probabilistic hough transforms

 /*vector<Vec4i> lines;
  HoughLinesP(dst, lines, 3,5*CV_PI/180, 100, 200, 1000 );
  for( size_t i = 0; i < lines.size(); i++ )
  {
    Vec4i l = lines[i];
    line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, CV_AA);
	std::cout  << l[0] << " " << l[1] << " " << l[2] << " " << l[3] << " " << rho << " " << "D" << " " << i;
std::cout << '\n';
  }*/
 
 			imshow("detected lines",cdst);	//showing the detected lines 
			imshow("windowName2",threshold);//showing the detected color 
			imshow("windowname",camerafeed1);//showing the normalized and blurred frames 
			imshow("windowName1",HSV);//showing the hsv frames 
			putld << cdst;//saving the frames
			putnorm << camerafeed1;
			putth << threshold;
			puthsv << HSV;		
			char c=waitKey(30);
            if (c==27)
            break;end = clock();
			cvReleaseImage(&camfed1);//releasing the pointers
			cvReleaseImage(&r);
			cvReleaseImage(&g);
			cvReleaseImage(&b);
		}
        cvReleaseCapture( &capture );//rreleasing the capture
 		return 0;
 		fclose(fp);
	}



/*serial communication code*/








/*#include<iostream>
#include<fstream>
#include<cstdlib>
#include <string.h>
#include "serial_talk.h"
*/


//using namespace std;

/*

void port_initialize(char const *portname, char const *baud)
{
	char tty_settings[200];
	strcpy(tty_settings, "stty -F ");
	strcat(tty_settings, portname);
	strcat(tty_settings,  " cs8 ");
	strcat(tty_settings, baud);
	strcat(tty_settings,  " ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon -crtscts");

	cout << "BAUD rate: " << baud << endl;

	system(tty_settings);						//Activates the tty connection with the Arduino
	Arduino_Input.open(portname);				//Opens the tty connection as an ifstream
	Arduino_Output.open(portname);				//Opens the tty connection as an ofstream, not used in this example
	double Voltage;								//The Arduino is reading the voltage from A0
	long int Time = time(NULL);
	int i;

	while(time(NULL)-Time < 5){}	//Wait five seconds for the Arduino to start up
}

int send_via_port(void *message, char const *type, int size)
{

//	for(i = 0; i < 100;)
//	{
//		Time = time(NULL);
//		while(time(NULL)-Time < 1){}	//wait one second to get
//good numbers into the Arduino stream
//		while(!Arduino_Input.eof())	//while the eof flage isn't set
//		{
//			Arduino_Input >> Voltage;	//will set the
//error flag if not ready, will get a number from the Arduino stream if ready
//			cout << Voltage << endl;	//Output it to the cout
//stream
//			i++;	//Increament i, it is not known how many numbers
//I'll get at a time
//		}
//		Arduino_Input.clear();	//eof flag won't clear itself
//	}

//	Arduino_Input.close();	//Close the ifstream

	//char msg[] = "ADIOS!!!";
	int bla = 10;
	
	if (strcmp(type, "double") == 0) {
		Arduino_Output << *(double *)message << std::flush;
	}
	else if (strcmp(type, "int") == 0) {
		Arduino_Output << *(int *)message << std::flush;
	}
	else if (strcmp(type, "char") == 0) {
//		Arduino_Output << *(char *)message << std::flush;
		Arduino_Output.write((char const *)message, size);
	}
//	else {
//		Arduino_Output << endl;
//	}

//	Arduino_Output.close();	//Close the ofstream
	return(0);

} 

void *read_from_port(void *message)
{

//	for(i = 0; i < 100;)
//	{
//		Time = time(NULL);
//		while(time(NULL)-Time < 1){}	//wait one second to get
//good numbers into the Arduino stream
//		while(!Arduino_Input.eof())	//while the eof flage isn't set
//		{
//			Arduino_Input >> Voltage;	//will set the
//error flag if not ready, will get a number from the Arduino stream if ready
//			cout << Voltage << endl;	//Output it to the cout
//stream
//			i++;	//Increament i, it is not known how many numbers
//I'll get at a time
//		}
//		Arduino_Input.clear();	//eof flag won't clear itself
//	}

//	Arduino_Input.close();	//Close the ifstream

	//char msg[] = "ADIOS!!!";
	
	int count;
	char bla[15];
	for(count = 0; count < 1; count++)	//while the eof flage isn't set
	{	
		//if (Arduino_Input.eof() == 1) {
			//return 1;
		//}
		Arduino_Input.get(bla, 7);
		//cout << "blallala: " << bla << endl;
		*(double *)message = atof(bla);
		//if (strcmp(type, "double") == 0) {
			////cout << Arduino_Input;
			//Arduino_Input >> *(double *)message;
		//}
		//else if (strcmp(type, "int") == 0) {
			//Arduino_Input >> *(int *)message;
		//}
		//else if (strcmp(type, "char") == 0) {
			//Arduino_Input >> *(char *)message;
		//}
		//else {
			//return 1;
		//}
	}
	my_finish_flag = 1;

//	Arduino_Output.close();	//Close the ofstream
	Arduino_Input.clear();
	pthread_exit(NULL);
	//return(0);

} 

void port_destruct()
{
	Arduino_Input.close();	//Close the ifstream
	Arduino_Output.close();	//Close the ofstream
}*/
/*int main()
{
	port_initialize("/dev/ttyACM1","57600");
	cout<<"connected"<<endl;
	send_via_port((char *)"h","char",1);
	return 0;
}
*/




void cal_pid()
{
	errorx=n-setpoint;
	error_sum+=(int)errorx*sampletime;
	if(errorx<10)
	{ if(ex==false)
	pid_output=(int)(kp*errorx+ki*error_sum+kd*(errorx-prev_error)/sampletime);
	  else
        { pid_output=(int)(kp*errorx+ki*error_sum);
          ex=!ex;}
        }
	else
	{ if(ex==false)
	pid_output=(int)(kp*errorx+aki*error_sum+kd*(errorx-prev_error)/sampletime);
           else
        { pid_output=(int)(kp*errorx+aki*error_sum);	
          ex=!ex;}	
	}if(pid_output>55)
	 pid_output=55;	
	prev_error=errorx;
}


