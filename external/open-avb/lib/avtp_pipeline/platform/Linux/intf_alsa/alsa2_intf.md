ALSA2 interface {#alsa2_intf}
==============

# Description

ALSA interface module. An interface to connect AVTP streams to ALSA either as an audio source or sink.
This is different from alsa_intf in that it uses a different implementation of
the ALSA library.

<br>
# Interface module configuration parameters

All of the parameters allowed by alsa_intf is allowed here. These are the
additional parameters.

Name                      | Description
--------------------------|---------------------------
intf_nv_clock_source_timestamp_interval | When this interface is used to       \
                           generate reference clock ticks, this value is the   \
                           number of frames that must elapse between clock     \
                           ticks.
intf_nv_clock_source_timestamp_throwaway | When this interface is used to      \
                           generate reference clock ticks, this value is the   \
                           number of clock ticks to throw away. This is useful \
                           if the first few ticks are not steady               \
                           because, for example, the audio pipeline is not     \
                           full.
intf_nv_gen_clock_tick_on_consume_audio | Set this to 1 to generate reference  \
                           clock ticks when audio is consumed. A clock tick    \
                           will be generated when                              \
                           intf_nv_clock_source_timestamp_interval frames      \
                           have been consumed.
intf_nv_sync_to_clock_tick_on_tx_audio | When this interface is configured as  \
                           a talker, set this to 1 to sync the audio stream    \
                           to the reference clock ticks. Set this to 0 to      \
                           let the talker freerun.
intf_nv_clock_recovery_adjustment_range | When                                 \
                           intf_nv_sync_to_clock_tick_on_tx_audio is set to 1, \
                           the audio stream will be adjusted by 1 frame per    \
                           clock tick for each multiple of this value that the \
                           audio stream is ahead or behind the reference clock.\
                           For example, if this is set to 500, then the talker \
                           will insert one addition frame per clock tick for   \
                           every 500 frames that the stream is behind the      \
                           reference clock.

<br>

