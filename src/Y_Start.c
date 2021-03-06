/*
*	FileName:Y_Start.c
*	Version: 1.1	
*	Description: The file defines a framework about one 
*		specail way that is a start of a program.It
*		will create a child-process as real main-f-
*		unction.Then,the parent-process as guard-p-
*		rocess.
*	Created On:2016-2-16
*	Modified date:2016-2-17
*	Author:Sky
*
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/wait.h>
#include<sys/types.h>
#include "Y_Macro.h"
#include "Y_ChildProcess.h"
#include <X11/Xlib.h>

typedef struct {
	//
	//Your code here.
	//
} Y_sARG;

int main(int argc, char * argv []){
	
	pid_t cpid;
	pid_t wait_pid;
	int ret_status;
	Y_sARG arg;
	void *retval;


	XInitThreads();	

	cpid = fork();
	if (cpid == -1){
		ERROR_MSG("fpid<0","creating child-process failed!");
		return (PROGRAM_ERROR);
	}
	if (cpid == 0){/* Code executed by child */
#ifdef Y_DEBUG
		SHOW_MSG("Hello, I am child-process.");
		Y_PRINTF("childprocess: parent pid is %4ld,current pid is %4ld \n", (long)getppid(),(long)getpid());
#endif //Y_DEBUG

		//
		//Your code here.
		//
		
		/*We can use follow codes that makes childprocess waits for signals */
		/*
		if (argc == 1)
                   pause();
		*/

		//The fuction is the program entrance.
		retval = Y_ChildProcess((const void *) &arg);	

		//child-process returned.
		exit(EXIT_SUCCESS);
	}		
	else{/* Code executed by parent */

#ifdef Y_DEBUG
		//SHOW_MSG("Hello, I am parent-process.");
		Y_PRINTF("parentprocess: parent pid is %4ld,current pid is %4ld \n", (long)getppid(),(long)getpid());
#endif //Y_DEBUG
		

		//
		//Your code here.
		//
		
		//Thoes codes refence man wait()
		do{
			/*       
			*	WUNTRACED   also return if a child has  stopped  (but  not  traced  via
                   	*	ptrace(2)).   Status for traced children which have stopped
                   	*	is provided even if this option is not specified.
			*
       			*	WCONTINUED (since Linux 2.6.10)
                   	*	also return if a stopped child has been resumed by delivery
                   	*	of SIGCONT.
			*/


			wait_pid = waitpid(cpid, &ret_status, WUNTRACED | WCONTINUED);
			if (wait_pid == -1){
				
				ERROR_MSG("waitpid()", "Value of return is error(-1)");
				return	(PROGRAM_ERROR);	
			}		
			//we can use follow codes to deal childprocess-status by a signal
			/*
			                if (WIFEXITED(status)) {
                   				printf("exited, status=%d\n", WEXITSTATUS(status));
                   			} else if (WIFSIGNALED(status)) {
                       				printf("killed by signal %d\n", WTERMSIG(status));
                   			} else if (WIFSTOPPED(status)) {
                       				printf("stopped by signal %d\n", WSTOPSIG(status));
                  		 	} else if (WIFCONTINUED(status)) {
                       				printf("continued\n");
                   			}
			*/


		/* 	WIFEXITED(status)
              	*	returns true if the child terminated normally, that is, by call‐
              	*	ing exit(3) or _exit(2), or by returning from main().
		*
       		*	WIFSIGNALED(status)
              	*	returns true if the child process was terminated by a signal.
		*/
		}while (!WIFEXITED(ret_status) && !WIFSIGNALED(ret_status));
		
#ifdef Y_DEBUG
		SHOW_MSG("All-process-returned.");
#endif //Y_DEBUG

		//parent-process returned.
		exit (PROGRAM_END);
	}
}
