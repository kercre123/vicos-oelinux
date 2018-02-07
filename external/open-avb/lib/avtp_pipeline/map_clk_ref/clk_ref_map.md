Clock Reference Mapping {#clk_ref_map}
==========================

# Description

Uncompressed audio mapping module conforming to AVB 61883-6 encapsulation.

# Mapping module configuration parameters

Name                | Description
--------------------|---------------------------
map_nv_item_count   |The number of media queue elements to hold.
map_nv_tx_rate or map_nv_tx_interval | Transmit interval in packets per second.\
                     0 = default for talker class.
map_nv_packing_factor|The number of packets of timestamps to accept in one     \
                      media queue item.
map_nv_timestamp_interval|The number of events (e.g. audio samples) that should\
                     occur between each timestamp.
map_nv_timestamps_per_packet|The number of timestamps to include in each       \
                     packet.
map_nv_pull_multiplier|The value from 1722-D15 for the multiplier for the base \
                     frequency to arrive at the nominal sampling rate. <br>    \
                     0 : Multiply by 1 <br>                                    \
                     1 : Multiply by 1/1.001 <br>                              \
                     2 : Multiply by 1.001 <br>                                \
                     3 : Multiply by 24/25 <br>                                \
                     4 : Multiply by 25/24 <br>                                \
                     5 : Multiply by 1/8
map_nv_base_frequency|The nominal base sampling rate in hertz. This is         \
                     multiplied by the pull value to arrive at the nominal     \
                     sampling rate.
map_nv_crf_type     |The of the CRF stream <br>                                \
                     0 : User Specified <br>                                   \
                     1 : Audio Sample Timestamp <br>                           \
                     2 : Video Frame Timestamp <br>                            \
                     3 : Video Line Sync Timestamp <br>                        \
                     4 : Machine Cycle Timestamp

