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
#include <sys/types.h>

/**
 * Function daemon init
 * This function will: Open the Socket and allocate memory
 * @return 0 on success
 */
int vl53l1_daemon_init();

/**
 * Function daemon deinit
 * This function will: Close the Socket and deallocate memory
 * @return 0 on success
 */
int vl53l1_daemon_deinit();

/**
 * Function daemon run.
 * This function will start the listening from netlink channel and it never
 * ends. Use this function inside a thread to avoid blocking.
 */
void vl53l1_daemon_run();
