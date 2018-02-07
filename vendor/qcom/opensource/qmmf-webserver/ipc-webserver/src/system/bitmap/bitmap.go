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

package bitmap

import (
    "fmt"
)

// The Max Size is 0x01 << 32 at present(can expand to 0x01 << 64)
const BitmapSize = 0x01 << 30

type Bitmap struct {
    data []byte
    bitsize uint64
    maxpos uint64
}

func NewBitmap() *Bitmap {
    return NewBitmapSize(BitmapSize)
}

func NewBitmapSize(size int) *Bitmap {
    if size == 0 || size > BitmapSize {
        size = BitmapSize
    } else if remainder := size % 8; remainder != 0 {
        size += 8 - remainder
    }

    return &Bitmap{data: make([]byte, size>>3), bitsize: uint64(size - 1)}
}

func (this *Bitmap) SetBit(offset uint64, value uint8) bool {
    index, pos := offset/8, offset%8

    if this.bitsize < offset {
        return false
    }

    if value == 0 {
        this.data[index] &^= 0x01 << pos
    } else {
        this.data[index] |= 0x01 << pos

        if this.maxpos < offset {
            this.maxpos = offset
        }
    }

    return true
}

func (this *Bitmap) GetBit(offset uint64) uint8 {
    index, pos := offset/8, offset%8

    if this.bitsize < offset {
        return 0
    }

    return (this.data[index] >> pos) & 0x01
}

func (this *Bitmap) Maxpos() uint64 {
    return this.maxpos
}

func (this *Bitmap) String() string {
    var maxTotal, bitTotal uint64 = 100, this.maxpos + 1

    if this.maxpos > maxTotal {
        bitTotal = maxTotal
    }

    numSlice := make([]uint64, 0, bitTotal)

    var offset uint64
    for offset = 0; offset < bitTotal; offset++ {
        if this.GetBit(offset) == 1 {
            numSlice = append(numSlice, offset)
        }
    }

    return fmt.Sprintf("%v", numSlice)
}

func (this *Bitmap) Find_first_zero_bit() uint64 {
    var maxTotal, bitTotal uint64 = 100, this.maxpos + 1

    if this.maxpos > maxTotal {
        bitTotal = maxTotal
    }

    var offset uint64
    for offset = 0; offset < bitTotal; offset++ {
        if this.GetBit(offset) == 0 {
            break
        }
    }
    return offset
}
