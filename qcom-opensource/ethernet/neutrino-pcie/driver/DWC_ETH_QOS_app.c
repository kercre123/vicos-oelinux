/* DWC_ETH_QOS_app.c: DWC_ETH_QOS app kernel module to open Neutrino Ethernet interface. */
/*
	Copyright (c) 2017, The Linux Foundation. All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are
	met:
		* Redistributions of source code must retain the above copyright
		  notice, this list of conditions and the following disclaimer.
		* Redistributions in binary form must reproduce the above
		  copyright notice, this list of conditions and the following
		  disclaimer in the documentation and/or other materials provided
		  with the distribution.
		* Neither the name of The Linux Foundation nor the names of its
		  contributors may be used to endorse or promote products derived
		  from this software without specific prior written permission.

	THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
	WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
	MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
	ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
	BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
	CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
	SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
	BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
	WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
	OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
	IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/rtnetlink.h>
#include <linux/if.h>


static int __init DWC_ETH_QOS_app_init (void)
{
	struct net_device *dev;

	rtnl_lock();
	for_each_netdev(&init_net, dev) {
		if(strncmp ("DWC_ETH_QOS", netdev_drivername(dev), 11) == 0)
			if (dev_change_flags(dev, dev->flags | IFF_UP) < 0)
				pr_err("DWC_ETH_QOS_app_init: Failed to open %s\n", dev->name);
	}
	rtnl_unlock();

	printk("\r\nCall DWC_ETH_QOS_open function for test purpose\r\n");
	return 0;
}


static void __exit DWC_ETH_QOS_app_cleanup (void)
{
	return;
}


module_init(DWC_ETH_QOS_app_init);
module_exit(DWC_ETH_QOS_app_cleanup);
