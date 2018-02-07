Clock Reference interface {#clk_ref_intf}
==============

# Description

Clock Reference Format (CRF) stream interface module.
An interface to connect CRF streams as a clock reference source or sink.

<br>
# Interface module configuration parameters

Name                      | Description
--------------------------|---------------------------
intf_nv_timestamp_usec_delay |The number of microseconds to add to each        \
                           CRF timestamp before adding it to the packet.       \
                           This number must be larger than the time between    \
                           each CRF packet.
<br>
# Notes

