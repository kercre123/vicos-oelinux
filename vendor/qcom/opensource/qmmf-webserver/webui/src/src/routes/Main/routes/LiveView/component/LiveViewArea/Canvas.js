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

'use strict'

const map = {
  'browser': { width: 960, height: 540 },
  '480P': { width: 640, height: 480 },
  '720P': { width: 1280, height: 720 },
  '1080P': { width: 1920, height: 1080 },
  '4K': { width: 3840, height: 2160 },
  'horizontalScale': 960 / 10000,
  'verticalScale': 540 / 10000,
}

class Frame {
  constructor (o, type, context) {
    this.id = o.id
    this.name = o.display_name
    this.ct = context
    this.type = type
    this.x = o.position.x * map.horizontalScale
    this.y = o.position.y * map.verticalScale
    this.width = o.position.width * map.horizontalScale
    this.height = o.position.height * map.verticalScale
    this.draw()
  }

  inRange({ x, y }) {
    return x > this.x && x < (this.x + this.width) && y > this.y && y < (this.height +  this.y)
  }

  drawNr () {
    this.ct.strokeStyle = 'white'
    this.ct.lineWidth = 4
    this.ct.strokeRect(this.x, this.y, this.width, this.height)
  }

  drawRf () {
    this.ct.strokeStyle = 'green'
    this.ct.lineWidth = 4
    this.ct.strokeRect(this.x, this.y, this.width, this.height)
  }

  drawFr () {
    this.ct.fillStyle = 'green'
    this.ct.font = '20px/1 arial'
    const width = this.ct.measureText(this.name).width
    this.ct.fillRect(this.x - 2, this.y - 30, width + 8, 30)
    this.ct.textBaseline = 'top'
    this.ct.fillStyle = 'white'
    this.ct.fillText(this.name, this.x + 2, this.y - 26)
    this.ct.strokeStyle = 'green'
    this.ct.lineWidth = 4
    this.ct.strokeRect(this.x, this.y, this.width, this.height)
  }

  drawMd () {
    this.ct.fillStyle = '#99d9ea'
    this.ct.font = '20px/1 arial'
    const mdWidth = this.ct.measureText(this.name).width
    this.ct.fillRect(this.x - 2, this.y - 30, mdWidth + 8, 30)
    this.ct.textBaseline = 'top'
    this.ct.fillStyle = 'white'
    this.ct.fillText(this.name, this.x + 2, this.y - 26)
    this.ct.strokeStyle = '#99d9ea'
    this.ct.lineWidth = 4
    this.ct.strokeRect(this.x, this.y, this.width, this.height)
  }

  draw () {
    switch (this.type) {
      case NR:
        this.drawNr()
        break
      case RF:
        this.drawRf()
        break
      case FR:
        this.drawFr()
        break
      case MD:
        this.drawMd()
        break
      default:
        break
    }
  }
}

class Frames {
  constructor () {
    this.canvas = document.createElement('canvas')
    this.canvas.width = this.width = map['browser'].width
    this.canvas.height = this.height = map['browser'].height
    this.context = this.canvas.getContext('2d')
    this.horizontalScale = this.width / 10000
    this.verticalScale = this.height / 10000
    this.list = new Map()
  }

  onMessage (e) {
    const data = JSON.parse(e)

    if (!data.objects || data.objects.length === 0) {
      this.clear ()
      return
    }
    this.deal(data)
  }

  setFrame (o, type) {
    this.list.set(o.id, new Frame(o, type, this.context))
  }

  clear () {
    this.context.clearRect(0, 0, this.width, this.height)
    this.list.clear()
  }
}

const FR = 'FACE_RECOGNITION'
const RF = 'REGISTER_FACE'
const NR = 'NORMAL'
const MD = 'MOTION_DETECTION'

class FrCanvas extends Frames {
  constructor () {
    super()
    this.registerId = null
  }

  onClick (point) {
    this.list.forEach(item => {
      if (item.inRange(point) && item.type === NR) {
         this.registerId = item.id
      }
      else if (item.inRange(point) && this.registerId == item.id) {
         this.registerId = null
      }
    })
  }

  clearRegisterId() {
    this.registerId = null
  }

  deal (data) {
    this.clear()
    const objects = data.objects
    objects.forEach(o => {
      let type
      if (o.type === 3) {
      if (o.display_name === 'Face recognition rule') {
        return
      }
      if (this.registerId && o.id === this.registerId && o.display_name ==='') {
        type = RF
      } else if (o.display_name !== '') {
        type = FR
      } else {
        type = NR
      }
      this.setFrame(o, type, this.context)
    }
  })
  }
}

class MdCanvas extends Frames {
  constructor () {
    super()
  }

  deal (data) {
    this.clear()
    const objects = data.objects
    objects.forEach(o => {
      let type
      if (o.type === 0) {
      type = MD
      this.setFrame(o, type, this.context)
    }
  })
  }
}

export default class Canvas {
  constructor (id, video, videoSettings) {
    let resolution =  videoSettings ? videoSettings.toUpperCase() : '1080P'
    console.log(resolution)
    this.flush = false
    this.video = video
    this.canvas = document.getElementById(id)
    this.context = this.canvas.getContext('2d')
    this.sourceHeight = map[resolution].height
    this.sourceWidth = map[resolution].width
    this.canvas.width = this.width = map['browser'].width
    this.canvas.height = this.height = map['browser'].height
    this.fr = new FrCanvas()
    this.md = new MdCanvas()
    this.showFr = false
    this.showMd = false
    this.addEventListener()
  }

  addEventListener () {
    this.canvas.addEventListener('click', this.canvasClick.bind(this), false)
  }
  removeEventListener () {
    this.canvas.removeEventListener('click', this.canvasClick.bind(this))
  }

  canvasClick (evt) {
    const point = { x: evt.layerX, y: evt.layerY }
    this.fr.onClick(point)
  }

  drawVideo () {
    this.context.drawImage(this.video, 0, 0, this.sourceWidth, this.sourceHeight, 0, 0, this.width, this.height)
  }

  onFrMessage (e) {
    this.fr.onMessage(e)
  }

  clearRegisterId () {
    this.fr.clearRegisterId()
  }

  drawFr () {
    this.context.drawImage(this.fr.canvas, 0, 0)
  }

  clearFr () {
    this.fr.clear()
  }

  startDrawFr () {
    this.showFr = true
  }

  stopDrawFr () {
    this.clearFr()
    this.showFr = false
  }

  onMdMessage (e) {
    this.md.onMessage(e)
  }

  drawMd () {
    this.context.drawImage(this.md.canvas, 0, 0)
  }

  clearMd () {
    this.md.clear()
  }

  startDrawMd () {
    this.showMd = true
  }

  stopDrawMd () {
    this.clearMd()
    this.showMd = false
  }

  onCtMessage (e) {
    const data = JSON.parse(e)
    if (!data.objects || data.objects.length === 0) {
      return
    }
    if (data.type === 1) {
      layer.alert('Camera tampering detected!', { icon: 7, btn: false, title: 'information' })
    }
  }

  clearCt () {
    return
  }

  getRegisterFacePosition () {
    let ret
    this.fr.list.forEach(frame => {
      if (frame.id === this.fr.registerId) {
      ret = {
        x: Math.round(frame.x / this.width * this.sourceWidth),
        y: Math.round(frame.y / this.height * this.sourceHeight),
        width: Math.round(frame.width / this.width * this.sourceWidth),
        height: Math.round(frame.height / this.height * this.sourceHeight),
      }
    }
  })

    return ret
  }

  start () {
    this.drawVideo()
    if (this.showFr) {
      this.drawFr()
    }
    if (this.showMd) {
      this.drawMd()
    }
    this. stopAnimationFrame = window.requestAnimationFrame(this.start.bind(this))
  }

  stop () {
    this.removeEventListener()
    window.cancelAnimationFrame(this.stopAnimationFrame)
    this.context.clearRect(0, 0, this.width, this.height)
  }
}
