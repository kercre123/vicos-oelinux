/*
* Copyright (c) 2017, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above
*       copyright notice, this list of conditions and the following
*       disclaimer in the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of The Linux Foundation nor the names of its
*       contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
* ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

package smap

import (
    "crypto/sha1"
    "fmt"
    . "os/exec"
    "sync"
)

type State int

const (
    Normal    State = iota // value --> 0
    Preview                // value --> 1
    Recording              // value --> 2
    Vam
)

const (
    Es = iota
    Ts
)

type VConf struct {
    Resolution int
    Encode     int
    Bitrate    int
    Fps        int
}

type OvConf struct {
    Ov_type           int
    Ov_color          string
    Ov_position       int
    Ov_user_text      string
    Ov_start_x        int
    Ov_start_y        int
    Ov_width          int
    Ov_height         int
    Ov_image_location int
    Ov_box_name       string
    Ov_date           int
    Ov_time           int
}

type Track_Id struct {
    RtspId      int
    RecordingId int
    VamId       int
    AudioId     int
}

type Sess_id struct {
    PreviewId   int
    RecordingId int
    VamId       int
}

type Cparams struct {
    Tnr  string
    Shdr string
}

type Camera struct {
    Params Cparams
}

type VamConf struct {
    Counter int
    Status  bool
    Config  string
}

type Channel struct {
    Ch            int
    Sids          Sess_id
    PCounter      int
    VCounter      int
    VamConfig     [5]VamConf
    Status        State
    VideoConf     VConf
    TrackIds      Track_Id
    OvId          string
    OverlayStatus bool
    OverlayType   int
    OverlayConf   [3]OvConf
    Stream        int
}

type RtspProxyConf struct {
    ProxyCmd *Cmd
    Offset   uint64
    Port     string
    Status   bool
}

// define sess struct
type Session struct {
    Sess      string
    Channel   int
    PreStatus bool
    RecStatus bool
    VamStatus bool
    OvStatus  bool
    ProxyConf [5]RtspProxyConf
}

type CamMap map[string]*CamShard

type CamShard struct {
    items map[string]Camera
    lock  *sync.RWMutex
}

type ChMap map[string]*ChShard

type ChShard struct {
    items map[string]Channel
    lock  *sync.RWMutex
}

//type SessMap map[string]*SessShard

const (
    defaultShardCount uint8 = 32
)

type SessShard struct {
    items map[string]Session
    lock  *sync.RWMutex
}

type SessMap struct {
    shardCount uint8
    shards     []*SessShard
}

func New() *SessMap {
    return NewWithShard(defaultShardCount)
}

func NewWithShard(shardCount uint8) *SessMap {
    if !isPowerOfTwo(shardCount) {
        shardCount = defaultShardCount
    }
    m := new(SessMap)
    m.shardCount = shardCount
    m.shards = make([]*SessShard, m.shardCount)
    for i, _ := range m.shards {
        m.shards[i] = &SessShard{
            items: make(map[string]Session),
            lock:  new(sync.RWMutex),
        }
    }
    return m
}

func (m *SessMap) locate(key string) *SessShard {
    return m.shards[bkdrHash(key)&uint32((m.shardCount-1))]
}

func (m *SessMap) Get(key string) (value Session, ok bool) {
    shard := m.locate(key)
    shard.lock.Lock()
    value, ok = shard.items[key]
    shard.lock.Unlock()
    return
}

func (m *SessMap) Set(key string, value Session) {
    shard := m.locate(key)
    shard.lock.Lock()
    shard.items[key] = value
    shard.lock.Unlock()
}

// Removes an item
func (m *SessMap) Delete(key string) {
    shard := m.locate(key)
    shard.lock.Lock()
    delete(shard.items, key)
    shard.lock.Unlock()
}

func (m *SessMap) Size() int {
    size := 0
    for _, shard := range m.shards {
        shard.lock.Lock()
        size += len(shard.items)
        shard.lock.Unlock()
    }
    return size
}

func (m *SessMap) Flush() int {
    size := 0
    for _, shard := range m.shards {
        shard.lock.Lock()
        size += len(shard.items)
        shard.items = make(map[string]Session)
        shard.lock.Unlock()
    }
    return size
}

const seed uint32 = 131

func bkdrHash(str string) uint32 {
    var h uint32
    for _, c := range str {
        h = h*seed + uint32(c)
    }
    return h
}

func isPowerOfTwo(x uint8) bool {
    return x != 0 && (x&(x-1) == 0)
}

//*********************************************************************************//
func ChMap_New() ChMap {
    c := make(ChMap, 256)
    for i := 0; i < 256; i++ {
        c[fmt.Sprintf("%02x", i)] = &ChShard{
            items: make(map[string]Channel),
            lock:  new(sync.RWMutex),
        }
    }
    return c
}

func (c ChMap) ChMap_Get(key string) (value Channel, ok bool) {
    shard := c.ChMap_GetShard(key)
    shard.lock.RLock()
    defer shard.lock.RUnlock()
    value, ok = shard.items[key]
    return
}

func (c *ChMap) GetRef(key string) (value Channel, ok bool) {
    shard := c.ChMap_GetShard(key)
    shard.lock.Lock()
    value, ok = shard.items[key]
    shard.lock.Unlock()
    return
}

func (c ChMap) ChMap_Set(key string, data Channel) {
    shard := c.ChMap_GetShard(key)
    shard.lock.Lock()
    defer shard.lock.Unlock()
    shard.items[key] = data
}

func (c ChMap) ChMap_GetShard(key string) (shard *ChShard) {
    hasher := sha1.New()
    hasher.Write([]byte(key))
    shardKey := fmt.Sprintf("%x", hasher.Sum(nil))[0:2]
    return c[shardKey]
}

func (c ChMap) ChMap_Remove(key string) {
    shard := c.ChMap_GetShard(key)
    shard.lock.Lock()
    defer shard.lock.Unlock()
    delete(shard.items, key)
}

//*************************************************************************//
func CamMap_New() CamMap {
    c := make(CamMap, 16)
    for i := 0; i < 16; i++ {
        c[fmt.Sprintf("%02x", i)] = &CamShard{
            items: make(map[string]Camera),
            lock:  new(sync.RWMutex),
        }
    }
    return c
}

func (c CamMap) CamMap_Get(key string) (value Camera, ok bool) {
    shard := c.CamMap_GetShard(key)
    shard.lock.RLock()
    defer shard.lock.RUnlock()
    value, ok = shard.items[key]
    return
}

func (c CamMap) CamMap_Set(key string, data Camera) {
    shard := c.CamMap_GetShard(key)
    shard.lock.Lock()
    defer shard.lock.Unlock()
    shard.items[key] = data
}

func (c CamMap) CamMap_GetShard(key string) (shard *CamShard) {
    hasher := sha1.New()
    hasher.Write([]byte(key))
    shardKey := fmt.Sprintf("%x", hasher.Sum(nil))[0:2]
    return c[shardKey]
}
