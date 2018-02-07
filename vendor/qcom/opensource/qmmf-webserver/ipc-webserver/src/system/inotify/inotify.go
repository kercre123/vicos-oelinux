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

package inotify

/*
#include <stdlib.h>
#include <unistd.h>
#include <linux/limits.h>
#include <sys/inotify.h>

static inline const char *ev_name(const struct inotify_event *ev)
{
    return ev->name;
}

static inline const struct inotify_event *
        ev_next(const struct inotify_event *ev)
{
    return (const struct inotify_event *)
            ((const char *)ev + sizeof(*ev) + ev->len);
}
*/
import "C"
import "unsafe"

func CBytes(b []byte) unsafe.Pointer {
    p := C.malloc(C.size_t(len(b)))
    pp := (*[1<<30]byte)(p)
    copy(pp[:], b)
    return p
}

func CChars(b []byte) unsafe.Pointer {
    p := C.malloc(C.size_t(len(b) + 1))
    pp := (*[1<<30]byte)(p)
    copy(pp[:], b)
    pp[len(b)] = 0
    return p
}

// func InotifyAdd(dirs []string)

// func InotifyLoop(dirs []string, notify func([]string))

func InotifyLoop(dirs []string, notify func([]string, []string, []string)) {
    wd_to_dir := map[C.int]string{}
    mask_to_event := map[C.uint32_t]string{}

    mask_to_event[C.IN_CLOSE_WRITE] = "CREATE"
    mask_to_event[C.IN_DELETE] = "DELETE"

    ifd := C.inotify_init1(C.IN_CLOEXEC)
    defer C.close(ifd)

    for _, dir := range dirs {
        _dir := (*C.char)(CChars([]byte(dir)))
        wd := C.inotify_add_watch(ifd, _dir,
                    C.IN_CLOSE_WRITE | C.IN_DELETE)
        C.free(unsafe.Pointer(_dir))
        wd_to_dir[wd] = dir
    }

    ev := (*C.struct_inotify_event)(C.malloc(
            C.sizeof_struct_inotify_event + C.NAME_MAX + 1))
    defer C.free(unsafe.Pointer(ev))

    for {
        _ev := ev
        files := []string{}
        dirs := []string{}
        events := []string{}

        nleft := C.read(ifd, unsafe.Pointer(ev),
                C.sizeof_struct_inotify_event + C.NAME_MAX + 1)
        for nleft != 0 {
            nleft -= C.sizeof_struct_inotify_event +
                            (C.ssize_t)(_ev.len)
            files = append(files, C.GoString(C.ev_name(_ev)))
            dirs = append(dirs, wd_to_dir[_ev.wd])
            events = append(events, mask_to_event[_ev.mask])
            _ev = C.ev_next(_ev)
        }
        notify(files, dirs, events)
    }
}
