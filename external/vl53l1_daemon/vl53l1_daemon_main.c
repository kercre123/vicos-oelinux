/*******************************************************************************
 Copyright (C) 2016, STMicroelectronics International N.V.
 All rights reserved.

 Use and Redistribution are permitted only in accordance with licensing terms at www.st.com
 and provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of STMicroelectronics nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROTECTED BY STMICROELECTRONICS PATENT AND COPYRIGHTS.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
 NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
 IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF TCLOCK_MONOTICHE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include <pthread.h>
#include <unistd.h>
#include <stdio.h>

#include "vl53l1_daemon.h"

#define nl_err(fmt, ...) fprintf(stderr, "ERROR : " fmt "\n", ##__VA_ARGS__)
#define nl_inf(fmt, ...) fprintf(stderr, "INFO  : " fmt "\n", ##__VA_ARGS__)
#ifdef DEBUG
#	define nl_dbg(fmt, ...) fprintf(stderr, "DEBUG :  " fmt "\n", ##__VA_ARGS__)
#else
#	define nl_dbg(fmt, ...) (void)0
#endif

/* This code is just an example of how to run daemon with thread
 * if the RUN_THREAD_EXAMPLE is set then you can use it without thread */
//#define RUN_THREAD_EXAMPLE

int main(int argc, char **argv)
{
	int rc;
#ifdef RUN_THREAD_EXAMPLE
	pthread_t       thread1;
#endif
	int  iret1;

	nl_inf("Daemon init\n");
	rc = vl53l1_daemon_init();


#ifndef RUN_THREAD_EXAMPLE
	if (!rc) {
		vl53l1_daemon_run();
	}
#else

	if (!rc) {
		iret1 = pthread_create(&thread1,NULL,(void*)&vl53l1_daemon_run,NULL);
		nl_inf("thread created: Daemon running\n");
	}

	sleep(10);
	nl_inf("thread cancel request\n");
	pthread_cancel(thread1);
	nl_inf("wait x thread join\n");
	pthread_join( thread1, NULL);
	nl_inf("thread end\n");

	nl_inf("Daemon deinit\n");
	vl53l1_daemon_deinit();
	nl_inf("Daemon END\n");
#endif

	return rc;
}
