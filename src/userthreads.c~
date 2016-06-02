#include "userthreads.h"
#include "Y_Macro.h"
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>
#include <sys/types.h>

/*//this is a way of thread-implement 
void * thread1(const void * arg){//get video from camera
	void * a;
#ifdef Y_DEBUG
	SHOW_MSG("This is Thread1.\n");
#endif //Y_DEBUG
	return a;
}
*/

unsigned char endFlag=0x00;
CvPoint BoxPoint[4];

void on_trackbar(int pos);//declare call-back fuction
CvPoint * DrawBox(CvBox2D box,IplImage* img);
CvPoint getRectCenterPoint(CvPoint point[]);
void Sleep(int ms);
void Sleep(int ms)  
{  
    struct timeval delay;  
    delay.tv_sec = 0;  
    delay.tv_usec = ms * 1000; // 20 ms  
    select(0, NULL, NULL, NULL, &delay);  
}  


IplImage *CameraSrcImage = NULL;//frame-source 
IplImage *SrcImage = NULL;//frame-source 
IplImage *SrcFrame = NULL;//frame-source 
IplImage *GrayFrame = NULL;//GrayImage
IplImage *CpyGrayFrame = NULL;//GrayImage
IplImage *BinaryFrame = NULL;//BinaryImage
IplImage *CpyBinaryFrame = NULL;//BinaryImage
IplImage *CpyBinaryFrame1 = NULL;//BinaryImage for circle
IplImage *OutFrame = NULL;//BinaryImage
IplImage *SMHFrame = NULL;//BinaryImage
IplImage *SMHFrame_Af = NULL;//BinaryImage
IplImage *SMHFrame_Af1 = NULL;//BinaryImage


pthread_mutex_t CopySrcMutex;
pthread_mutex_t CopyGrayMutex;
pthread_mutex_t CopyBinaryMutex;


CvSeq *CircleSeq = NULL;
CvSeq *ContoursSeq = NULL;    
CvSeq *ContoursSeq_tmp = NULL; 
CvSeq *ApproxSeq = NULL;

CvMemStorage *ContoursStorage = NULL; //Contours memory
CvMemStorage *outStorage = NULL;
CvMemStorage *CircleStorage = NULL;
CvMemStorage *boxStorage = NULL;
CvMemStorage *ApproxStorage = NULL;

int NumContours = 0;
CvBox2D cvbox;
CvSize Imagesz;

CvPoint PointOfCenter;
				

CvPoint smh_act[4];
float smh_f[4];

			
CvPoint2D32f corners[6]; 
int cornerCount = 0;
int currenttime_mh[2];

#define PI 3.14159


//*************************************************************************************************
void thread0( void * arg){//get video from camera
	
	/*pthread_mutexattr_t attr;
  	pthread_mutexattr_init(&attr); //~necessary, or weird EINVAL error occurs when operating on the mutex
  	pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);*/
	pthread_mutex_init(&CopySrcMutex,NULL);
	pthread_mutex_init(&CopyGrayMutex, NULL);
	pthread_mutex_init(&CopyBinaryMutex, NULL);
	


	//pthread_mutexattr_destroy(&attr);
	SHOW_MSG("Init Mutex Completed!\n");
#ifdef Y_DEBUG
	SHOW_MSG("This is Thread0.\n");
#endif //Y_DEBUG
	Y_THREAD_EXIT(0);
}


//*************************************************************************************************

void  thread1( void * arg){//get video from camera

#ifdef Y_DEBUG
	Y_PRINTF("thread1: parent pid is %4ld,current pid is %4lu \n", (long)getppid(),pthread_self());
#endif
	
	Imagesz.width = 600;  
    Imagesz.height = 500; 
	
	SrcImage = cvCreateImage(Imagesz, IPL_DEPTH_8U, 3);
	CvCapture* pCapture = cvCreateCameraCapture(-1);

	//cvNamedWindow("Camera", 1);

	while(1)
  	{

      	CameraSrcImage=cvQueryFrame( pCapture );

		if(!CameraSrcImage)
			continue;
		
		pthread_mutex_lock(&CopySrcMutex);//prevent SrcImage is changed ,when we are using  it
		cvResize(CameraSrcImage,SrcImage,CV_INTER_AREA);
		pthread_mutex_unlock(&CopySrcMutex);

      	//cvShowImage("Camera",SrcImage);

		Sleep(33);
		//ascii=27 "Esc key"
		//there must be cvWaitKey()&0xff == ascii
      		//if( (cvWaitKey(33)&0xff) == 'q')
			//break;
		if (endFlag == 0x03)
			break;
  	}
	
	
	cvReleaseCapture(&pCapture);
	cvReleaseImage(&SrcImage); 
	cvDestroyWindow("Camera");
#ifdef Y_DEBUG
	SHOW_MSG("This is Thread1.\n");
#endif //Y_DEBUG
	
	Y_THREAD_EXIT(0);
}


//*************************************************************************************************
void  thread2( void * arg){

#ifdef Y_DEBUG
	Y_PRINTF("thread2: parent pid is %4ld,current pid is %4lu \n", (long)getppid(),pthread_self());
#endif


	while(SrcImage==NULL){}//wait datas from thread1

	//cvNamedWindow("GrayImage", 1);

	SrcFrame = cvCreateImage(Imagesz, IPL_DEPTH_8U, 3);
	GrayFrame = cvCreateImage(Imagesz, IPL_DEPTH_8U, 1);

	while(1){


		//void cvCopy( const CvArr* src, CvArr* dst, const CvArr* mask=NULL );
		pthread_mutex_lock(&CopySrcMutex);

		cvCopy(SrcImage,SrcFrame,NULL);

		pthread_mutex_unlock(&CopySrcMutex);


		//convert to gray image

		pthread_mutex_lock(&CopyGrayMutex);

		cvCvtColor(SrcFrame, GrayFrame, CV_BGR2GRAY); 

		pthread_mutex_unlock(&CopyGrayMutex);


		if(!GrayFrame)
			continue;

      		//cvShowImage("GrayImage",GrayFrame);


		Sleep(33);
		if (endFlag == 0x02)
		{
			endFlag = 0x03;
			break;	
		}
				
	}

	
	cvDestroyWindow("GrayImage");
	cvReleaseImage(&SrcFrame); 
	cvReleaseImage(&GrayFrame);
#ifdef Y_DEBUG
	SHOW_MSG("This is Thread2.\n");
#endif //Y_DEBUG

	Y_THREAD_EXIT(0);
}


//*************************************************************************************************
void  thread3( void * arg){//get video from camera


#ifdef Y_DEBUG
	Y_PRINTF("thread3: parent pid is %4ld,current pid is %4lu \n", (long)getppid(),pthread_self());
#endif

	while(GrayFrame==NULL){}//wait datas from thread2

	cvNamedWindow("BinaryImage", 1);
	
	CpyGrayFrame = cvCreateImage(Imagesz, IPL_DEPTH_8U, 1);
	BinaryFrame = cvCreateImage(Imagesz, IPL_DEPTH_8U, 1);

	

//create a trackbar
	int nThreshold = 196;
	cvCreateTrackbar("ThresholdValue", "BinaryImage", &nThreshold, 254, on_trackbar); 
	 
	

	while(1){
		//void cvCopy( const CvArr* src, CvArr* dst, const CvArr* mask=NULL );

		pthread_mutex_lock(&CopyGrayMutex);
		cvCopy(GrayFrame,CpyGrayFrame,NULL);
		pthread_mutex_unlock(&CopyGrayMutex);

		on_trackbar(nThreshold);

		//ascii=27 "Esc key"
		//there must be cvWaitKey()&0xff == ascii
      		//if( (cvWaitKey(33)&0xff) == 'e')
		//	break;
		Sleep(33);
		if (endFlag==0x01)
		{
			endFlag = 0x02;
			break;
		}
	}
	

	cvReleaseImage(&CpyGrayFrame); 
	cvReleaseImage(&BinaryFrame);
	cvDestroyWindow("BinaryImage");
#ifdef Y_DEBUG
	SHOW_MSG("This is Thread3.\n");
#endif //Y_DEBUG
	Y_THREAD_EXIT(0);
}
//*************************************************************************************************
void on_trackbar(int pos){
	

	//only get image from thread2 ,we will go on 
	//while(CpyGrayFrame==NULL){}
		
	//pthread_mutex_lock(&CallBackMutex);

//二值化
/*	
void cvThreshold( const CvArr* src,CvArr* dst,double threshold,double max_value,int threshold_type );
threshold_type=CV_THRESH_BINARY:如果 src(x,y)>threshold ,dst(x,y) = max_value; 否则,dst（x,y）=0;
*/   
	
	pthread_mutex_lock(&CopyBinaryMutex);

	cvThreshold(CpyGrayFrame, BinaryFrame, pos, 254, CV_THRESH_BINARY);

	//cvNot(BinaryFrame,BinaryFrame);

	pthread_mutex_unlock(&CopyBinaryMutex);
	cvShowImage("BinaryImage",BinaryFrame);

	//pthread_mutex_unlock(&CallBackMutex);
}

//*************************************************************************************************
CvPoint * DrawBox(CvBox2D box,IplImage* img) 
{ 
     CvPoint2D32f point[4]; 
      int i; 
      for ( i=0; i<4; i++) 
      { 
         point[i].x = 0; 
          point[i].y = 0; 
     } 
     cvBoxPoints(box, point); //计算二维盒子顶点 
   
     for ( i=0; i<4; i++) 
     { 
         BoxPoint[i].x = (int)point[i].x; 
         BoxPoint[i].y = (int)point[i].y; 
     } 

#ifdef ENABLE_DRAW_BOX
     cvLine( img, BoxPoint[0], BoxPoint[1],CV_RGB(255,0,0), 1, 8, 0 ); 
     cvLine( img, BoxPoint[1], BoxPoint[2],CV_RGB(255,0,0), 1, 8, 0 ); 
     cvLine( img, BoxPoint[2], BoxPoint[3],CV_RGB(255,0,0), 1, 8, 0 ); 
     cvLine( img, BoxPoint[3], BoxPoint[0],CV_RGB(255,0,0), 1, 8, 0 ); 
#endif //ENABLE_DRAW_BOX


	return BoxPoint;
} 

CvPoint getRectCenterPoint(CvPoint point[]){
	
	CvPoint centerp;
	
	centerp.y = ((point+0)->y - (point+1)->y)/2 + (point+1)->y;

	centerp.x = ((point+2)->x - (point+1)->x)/2 + (point+1)->x;

	return centerp;
	
}


static float CarmackSqrt (float x)
{
       float xhalf = 0.5f * x;
         
       int i = *(int*)&x;           // get bits for floating VALUE 
       i = 0x5f3759df - (i>>1);     // gives initial guess y0
       x = *(float*)&i;             // convert bits BACK to float
       x = x*(1.5f - xhalf*x*x);    // Newton step, repeating increases accuracy
       x = x*(1.5f - xhalf*x*x);    // Newton step, repeating increases accuracy
       x = x*(1.5f - xhalf*x*x);    // Newton step, repeating increases accuracy
       return (1 / x);
}


void DealWithCorner(CvPoint point){

	//float l = CarmackSqrt( ( (PointOfCenter.x-point->x)*(PointOfCenter.x-point->x) + (PointOfCenter.y-point->y)*(PointOfCenter.y-point->y) ) );
	float l = (PointOfCenter.x-point.x)*(PointOfCenter.x-point.x) + (PointOfCenter.y-point.y)*(PointOfCenter.y-point.y);
	float tmp;
	CvPoint tmp_p;

	for (int i = 0; i < 3; i++){
		
		if (l > smh_f[i]){
			tmp = smh_f[i];
			smh_f[i] = l;
			l = tmp;
		
			tmp_p.x = smh_act[i].x;
			smh_act[i].x = point.x;
			point.x = tmp_p.x;

			tmp_p.y = smh_act[i].y;
			smh_act[i].y = point.y;
			point.y = tmp_p.y;
		}
	}
	
}



void getCurrentTime(CvPoint smh[],CvPoint Center){
	
	float k_m;
	double arctan_m;

	float k_h;
	double arctan_h;

	for (int i = 0; i < 2 ; i++){//move Coordinate system
		
		smh_act[i].x -= PointOfCenter.x;
		smh_act[i].y -= PointOfCenter.y;
	}
	//caculating min
	if ( 0 == smh_act[0].x ){//arctan_m=90 or 270
		
		if (0 > smh_act[0].y){
			
			currenttime_mh[0] = 0;		
		}
		else{

			currenttime_mh[0] = 30;			
		}
	}
	else{
		
		k_m = smh_act[0].y/smh_act[0].x;
		//output -pi/2~pi/2
		arctan_m = atan((double)k_m);
		if ( !k_m ){ //k_m = 0
		
			if (0 > smh_act[0].x){
				
				currenttime_mh[0] = 45;
			}
			else{

				currenttime_mh[0] = 15;
			}
		}
		else{//k_m != 0
			
			if (smh_act[0].y > 0){ 
			
					if ( arctan_m > 0){//quadrant 1
						
						currenttime_mh[0] = 15 + (arctan_m/PI)*30;			
					}
					else{////quadrant 2
						
						currenttime_mh[0] = 45 - (-arctan_m/PI)*30;						
					}
			}
			else{
				
					if ( arctan_m > 0){//quadrant 3
						
						currenttime_mh[0] = 45 + (arctan_m/PI)*30;			
					}
					else{////quadrant 4
						
						currenttime_mh[0] = (-arctan_m/PI)*30;						
					}
			}
		}

	}


	//caculating hour

	if ( 0 == smh_act[1].x ){//arctan_m=90 or 270
		
		if (0 > smh_act[1].y){
			
			currenttime_mh[1] = 12;		
		}
		else{

			currenttime_mh[1] = 6;			
		}
	}
	else{
		
		k_m = smh_act[1].y/smh_act[1].x;
		//output -pi/2~pi/2
		arctan_m = atan((double)k_m);
		if ( !k_m ){ //k_m = 0
		
			if (0 > smh_act[1].x){
				
				currenttime_mh[1] = 9;
			}
			else{

				currenttime_mh[1] = 3;
			}
		}
		else{//k_m != 0
			
			if (smh_act[1].y > 0){ 
			
					if ( arctan_m > 0){//quadrant 1
						
						currenttime_mh[1] = 3 + (arctan_m/PI)*6;			
					}
					else{////quadrant 2
						
						currenttime_mh[1] = 9 - (-arctan_m/PI)*6;						
					}
			}
			else{
				
					if ( arctan_m > 0){//quadrant 3
						
						currenttime_mh[1] = 9 + (arctan_m/PI)*6;			
					}
					else{////quadrant 4
						
						currenttime_mh[1] = (-arctan_m/PI)*6;						
					}
			}
		}

	}

	
}



//*************************************************************************************************
void  thread4(void * arg){//get video from camera



#ifdef Y_DEBUG
	Y_PRINTF("thread4: parent pid is %4ld,current pid is %4lu \n", (long)getppid(),0xffff&pthread_self());
#endif	


	while(BinaryFrame==NULL){}//wait datas from thread3

	cvNamedWindow("OutImage", 1);
	//cvNamedWindow("test", 1);
	
	CpyBinaryFrame = cvCreateImage(Imagesz, IPL_DEPTH_8U, 1);
	CpyBinaryFrame1 = cvCreateImage(Imagesz, IPL_DEPTH_8U, 1);
	OutFrame = cvCreateImage(Imagesz, IPL_DEPTH_8U, 3); 
	SMHFrame = 	cvCreateImage(Imagesz, IPL_DEPTH_8U, 1); 
	SMHFrame_Af = 	cvCreateImage(Imagesz, IPL_DEPTH_32F, 1); 
	SMHFrame_Af1 = 	cvCreateImage(Imagesz, IPL_DEPTH_32F, 1); 

	cvRectangle(SMHFrame, cvPoint(0, 0), cvPoint(OutFrame->width, OutFrame->height), CV_RGB(255, 255, 255), CV_FILLED,0,0);

	ContoursStorage = cvCreateMemStorage(0);
	boxStorage = cvCreateMemStorage(0);
	CircleStorage = cvCreateMemStorage(0);
	ApproxStorage = cvCreateMemStorage(0);


	while(1){

	pthread_mutex_lock(&CopyBinaryMutex);

	cvCopy(BinaryFrame,CpyBinaryFrame,NULL);

	pthread_mutex_unlock(&CopyBinaryMutex);

	cvCopy(CpyBinaryFrame,CpyBinaryFrame1,NULL);


//get contours

#ifdef WAY1_FOUND_CONTOURS

	NumContours =  cvFindContours(CpyBinaryFrame, ContoursStorage, &ContoursSeq, sizeof(CvContour), CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));

#else 

	NumContours =  cvFindContours(CpyBinaryFrame, ContoursStorage, &ContoursSeq, sizeof(CvContour), CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));//get inter-countours

#endif //WAY1_FOUND_CONTOURS

	cvRectangle(OutFrame, cvPoint(0, 0), cvPoint(OutFrame->width, OutFrame->height), CV_RGB(255, 255, 255), CV_FILLED,0,0);
	//printf("NumContours is %d \n",NumContours);  

#ifdef WAY1
	for (;ContoursSeq!=0;ContoursSeq=ContoursSeq->v_next){
		
		ContoursSeq_tmp = ContoursSeq;//backup ContoursSeq
		for(;ContoursSeq!=0;ContoursSeq=ContoursSeq->h_next)  
        	{
			cvbox =  cvMinAreaRect2(ContoursSeq,boxStorage);  
			DrawBox(cvbox,OutFrame);
			cvDrawContours(OutFrame, ContoursSeq, CV_RGB(255,0,0), CV_RGB(0, 255, 0), 0, 1, 0,cvPoint(0,0));
		}
		ContoursSeq = ContoursSeq_tmp;
	}


#else

	for (;ContoursSeq!=0;ContoursSeq=ContoursSeq->h_next){
		
	//	CvRect aRect = cvBoundingRect( ContoursSeq, 0 );   //get rectange width and height

		double tmparea=fabs(cvContourArea(ContoursSeq,CV_WHOLE_SEQ, 0));      //get area of coutours

        if ( /*((aRect.width/aRect.height) >= 0.8) ||*/ (tmparea < 900))    
        {    
            cvSeqRemove(ContoursSeq,0); //删除宽高比例小于设定值的轮廓     
            		   
        }    
		else{
			
			CvPoint * tmp;
			if (tmparea > 25000)//25000 is coming from caculation
			{
				cvbox =  cvMinAreaRect2(ContoursSeq,boxStorage);  
				tmp = DrawBox(cvbox,OutFrame);
				PointOfCenter = getRectCenterPoint(tmp);
				cvCircle (OutFrame, PointOfCenter, 10, CV_RGB(0, 0, 255), 2, 8, 0); 
				
			}
			else{

				//cvRectangle(SMHFrame, cvPoint(0, 0), cvPoint(OutFrame->width, OutFrame->height), CV_RGB(255, 255, 255), CV_FILLED,0,0);				
				//cvDrawContours(SMHFrame, ContoursSeq, CV_RGB(0,0,0), CV_RGB(0, 255, 0), 0, 1, CV_AA,cvPoint(0,0));
				//cvShowImage("test",SMHFrame);

//C: CvSeq* cvConvexHull2(const CvArr* input, void* hull_storage=NULL, int orientation=CV_CLOCKWISE, int return_points=0 )

				CvSeq * p = cvConvexHull2(ContoursSeq,ApproxStorage,CV_CLOCKWISE,1);
				cvDrawContours (OutFrame, p, CV_RGB(0,255,0),CV_RGB(0,0,255),0,1,0,cvPoint(0,0));
				CvPoint mmc;

				for (int i = 0; i < p->total; i++){
					
				 	CvPoint* tmdn = (CvPoint*)cvGetSeqElem(p,i); 		
					if ( ( fabs(mmc.x-tmdn->x) > 25) &&( fabs(mmc.y-tmdn->y) > 25) ){
						

				 		cvCircle (OutFrame, cvPoint( tmdn->x, tmdn->y ), 6, CV_RGB(0, 255, 0), 1, 8, 0);
						mmc.x = tmdn->x;
						mmc.y = tmdn->y;

						DealWithCorner(mmc);

					}
				}

				cvCircle (OutFrame, cvPoint( smh_act[0].x, smh_act[0].y ), 6, CV_RGB(0, 0, 0), 2, 8, 0);
				cvCircle (OutFrame, cvPoint( smh_act[1].x, smh_act[1].y ), 6, CV_RGB(0, 0, 0), 2, 8, 0);
				getCurrentTime(smh_act,PointOfCenter);
#ifdef Y_DEBUG

			char point[30];
			CvFont font;
      		cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,1,1,0,2,8);
      		
			sprintf(point,"Current Time is -- %d:%d",currenttime_mh[1],currenttime_mh[0]);
			cvPutText(OutFrame,point,cvPoint(30,30),&font,CV_RGB(0,255,0));


#endif//Y_DEBUG	
				cvDrawContours(OutFrame, ContoursSeq, CV_RGB(255,0,0), CV_RGB(0, 255, 0), 0, 1, 0,cvPoint(0,0));
			}		
		

			
		}

		

	}

#endif//WAY1

	for (int i = 0; i < 3; i++){
	
		smh_f[i] = 0;
		smh_act[i].x = 0;
		smh_act[i].y = 0;	
	}




/*
circle = cvHoughCircles( //cvHoughCircles函数需要估计每一个像素梯度的方向，
//因此会在内部自动调用cvSobel,而二值边缘图像的处理是比较难的
img1,
storage,
CV_HOUGH_GRADIENT,
1, //累加器图像的分辨率,增大则分辨率变小
18, //很重要的一个参数，告诉两个圆之间的距离的最小距离，如果已知一副图像，可以先行计
//算出符合自己需要的两个圆之间的最小距离。
100, //canny算法的阈值上限，下限为一半（即100以上为边缘点，50以下抛弃，中间视是否相连而
//定）
25, //决定成圆的多寡 ，一个圆上的像素超过这个阈值，则成圆，否则丢弃
32,//最小圆半径，这个可以通过图片确定你需要的圆的区间范围
45 //最大圆半径
);
*/

//get circle

#ifdef ENABLE_HOUGHCIRCLE

	CircleSeq = cvHoughCircles(CpyBinaryFrame1,CircleStorage,CV_HOUGH_GRADIENT,1,20,100,37,30,CpyBinaryFrame->height*0.5); 
	
	for (int i = 0; i < CircleSeq->total; i++){
		
		float* p = (float*)cvGetSeqElem (CircleSeq, i);
		cvCircle (OutFrame, cvPoint (cvRound(p[0]), cvRound(p[1])), cvRound(p[2]), CV_RGB(0, 0, 255), 1, 8, 0); 
	}

#endif//ENABLE_HOUGHCIRCLE






	cvShowImage("OutImage",OutFrame);



    if( (cvWaitKey(33)&0xff) == 'q'){
			
		endFlag = 0x01;
		break;
	}
			
	}
	


	cvReleaseMemStorage(&ContoursStorage);
	cvReleaseMemStorage(&boxStorage);
	cvReleaseMemStorage(&CircleStorage);
	cvReleaseMemStorage(&ApproxStorage);



	cvReleaseImage(&OutFrame); 
	cvReleaseImage(&CpyBinaryFrame); 
	cvReleaseImage(&CpyBinaryFrame1); 

	cvDestroyWindow("test");
	cvDestroyWindow("OutImage");
#ifdef Y_DEBUG
	SHOW_MSG("This is Thread4.\n");
#endif //Y_DEBUG
	Y_THREAD_EXIT(0);
}
//*************************************************************************************************

