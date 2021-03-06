/*
*	FileName:ChildProcess.c
*	Version: 1.1	
*	Description: The file defines the body of ChildProcess() 
*	Created On:2016-5-7
*	Modified date:
*	Author:Sky
*
*/
#include "Y_ChildProcess.h"



#define createThread(num,name,status) { \
		.threadnum=num,\
		.threadname=name##num,\
		.runstatus=status }

Ys_Thread ThreadTable[] = {
	
	createThread(0,thread,1),
	createThread(1,thread,1),
	createThread(2,thread,1),
	createThread(3,thread,1),
	createThread(4,thread,1)
};







void * Y_ChildProcess(const void * arg){

	void * retval;
	int retv;
	//
	//Your code here.
	//


	for ( int i = 0; i < ThreadTotalNum; i++){
		
		retv = Y_THREAD_CREATE(&ThreadTable[i].th_id,NULL,ThreadTable[i].threadname,ThreadTable[i].arg);
		if (retv!=0)
  		{
    			Y_PRINTF ("Create pthread %d error!\n",i+1);
			
    			return (void *)1;
  		}
	}
	
	for (int i = 0; i < ThreadTotalNum; i++){
			
		Y_THREAD_JOIN(ThreadTable[i].th_id, &retval);

    	Y_PRINTF("\n thread %d  retern is %d\n", i ,(int)retval);
	}


#ifdef Y_DEBUG
	SHOW_MSG("This is ChildProcess.");
#endif //Y_DEBUG
	
	//retval can be any type of point,only force-convertting to void *
	return (retval);
}
