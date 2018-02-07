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

import React, { Component, PropTypes } from 'react'
import './fileList.sass'
import { sendGetReplay, sendGetImage, sendDelImage, sendDelVideo } from 'ajax'
import Item from './Item'
import layer from 'layerUI'

class FileList extends Component {
  constructor (props) {
    super(props)
    this.state = {
      videoList: [],
      videoItemSelected: null,
      videoTotalPageNum: null,
      videoPageSelected: 1,
      videoDelFiles: [],
      imageList: [],
      imageItemSelected: null,
      imageTotalPageNum: null,
      imagePageSelected: 1,
      imageDelFiles: [],
      showDeleteBtn: false,
      isVideoList:false,
      isImageList:false,
    }
  }

  componentWillReceiveProps (nextProps) {
    if (nextProps.selected !== this.props.selected) {
      if (this.isCurrent('video')) {
        this.setState({
          videoDelFiles: []
        })
      }
      if (this.isCurrent('image')) {
        this.setState({
          imageDelFiles: []
        })
      }
    }
  }


  isCurrent = name => {
    return this.props.selected.indexOf(name) !== -1
  }

  onVideoItemClick = e => {
    if (this.state.showDeleteBtn) {
      return
    }
    const evt = Object.assign(e, { type: 'video' })
    this.setState({
      videoItemSelected: {
        filename: e.filename,
        url: e.url
      },
      imageItemSelected: null,
      showDeleteBtn: false
    })
    this.props.onClick(evt)
  }

  onImageItemClick = e => {
    if (this.state.showDeleteBtn) {
      return
    }
    const evt = Object.assign(e, { type: 'image' })
    this.setState({
      imageItemSelected: {
        filename: e.filename,
        url: e.url
      },
      videoItemSelected: null,
      showDeleteBtn: false
    })
    this.props.onClick(evt)
  }

  selectPage = e => {
    let pageNumber = parseInt(e.target.innerHTML, 10)
    if (this.isCurrent('video')) {
      sendGetReplay({ pageNumber }).then(data => {
        if (data.status) {
          let files = data.files
          if (files.length !== 0) {
            files.map((item, i) => {
              item.checked = false;
            })
          }
          this.setState({
            videoList: data.files,
            videoTotalPageNum: data.totalPageNum,
            videoPageSelected: pageNumber
          })
        }
      }).catch(e => console.log(e))
    } else {
      sendGetImage({ pageNumber }).then(data => {
        if (data.status) {
          let files = data.files
          if (files.length !== 0) {
            files.map((item, i) => {
              item.checked = false;
            })
          }
          this.setState({
            imageList: files,
            imageTotalPageNum: data.totalPageNum,
            imagePageSelected: pageNumber
          })
        }
      }).catch(e => console.log(e))
    }
  }

  showDeleteBtn = () => {
    if (this.isCurrent('video') && this.state.videoList.length ||
      this.isCurrent('image') && this.state.imageList.length) {
      const { videoList, imageList } = this.state
      videoList.map((item, i) => {
        item.checked = false
      })
      imageList.map((item, i) => {
          item.checked = false
      })
      if (this.state.showDeleteBtn) {
        this.setState({
          showDeleteBtn: !this.state.showDeleteBtn
        })
      } else {
        this.setState({
          showDeleteBtn: !this.state.showDeleteBtn
        })
      }
      this.setState({
        videoDelFiles: [],
        imageDelFiles: [],
        isVideoList: false,
        videoChecked: false,
        isImageList: false,
        imageChecked: false
      })
    }
  }

  deleteItem = e => {
    const { videoDelFiles, imageDelFiles } = this.state
    let reqData = {}
    if (this.isCurrent('video')) {
      if (videoDelFiles.length === 0) {
        return layer.msg('Please select the file you want to delete.', { icon: 5 })
      }
      reqData.files = videoDelFiles.map(item => {
        return { url: item }
      })
      return sendDelVideo(reqData).then(data => {
        let msg, icon
        if (data.status) {
          this.props.deleteSuccess()
          msg = 'Delete success！'
          icon = 6
        } else {
          msg = 'Delete Fail！Please try again later.'
          icon = 5
        }
        layer.msg(msg, { icon })
        return sendGetReplay({ pageNumber: 1 }).then(data => {
          if (data.status) {
            let files = data.files
            if (files.length !== 0) {
              files.map((item, i) => {
                item.checked = false;
              })
            }
            this.setState({
              videoList: data.files,
              videoTotalPageNum: data.totalPageNum,
              videoPageSelected: 1,
              videoItemSelected: null,
              videoDelFiles: [],
              showDeleteBtn: false,
              videoChecked:false,
              imageChecked:false
            })
          }
        }).catch(e => console.log(e))
      })
    } else {
      if (imageDelFiles.length === 0) {
        return layer.msg('Please select the file you want to delete.', { icon: 5 })
      }
      reqData.files = imageDelFiles.map(item => {
        return { url: item }
      })

      return sendDelImage(reqData).then(data => {
        let msg
        let icon
        if (data.status) {
          this.props.deleteSuccess()
          msg = 'Delete success！'
          icon = 6
        } else {
          msg = 'Delete Fail！Please try again later.'
          icon = 5
        }
        layer.msg(msg, { icon })
        return sendGetImage({ pageNumber: 1 }).then(data => {
          if (data.status) {
            let files = data.files
            if (files.length !== 0) {
              files.map((item, i) => {
                item.checked = false;
              })
            }
            this.setState({
              imageList: files,
              imageTotalPageNum: data.totalPageNum,
              imagePageSelected: 1,
              imageItemSelected: null,
              imageDelFiles: [],
              showDeleteBtn: false,
              videoChecked:false,
              imageChecked:false
            })
          }
        }).catch(e => console.log(e))
      })
    }
  }

  componentDidMount () {
    sendGetReplay({ pageNumber: 1 }).then(data => {
      if (data.status) {
        let files = data.files
        if (files.length !== 0) {
          files.map((item, i) => {
            item.checked = false;
          })
        }
        this.setState({
          videoList: files,
          videoTotalPageNum: data.totalPageNum
        })
      }
    }).catch(e => console.log(e))
    sendGetImage({ pageNumber: 1 }).then(data => {
      if (data.status) {
        let files = data.files
        if (files.length !== 0) {
          files.map((item, i) => {
            item.checked = false;
          })
        }
        this.setState({
          imageList: files,
          imageTotalPageNum: data.totalPageNum
        })
      }
    }).catch(e => console.log(e))
  }

  renderPage () {
    const {
      videoTotalPageNum, videoPageSelected,
      imageTotalPageNum, imagePageSelected
    } = this.state
    let totalPageNum = this.isCurrent('video') ?
      videoTotalPageNum : imageTotalPageNum
    let pageSelected = this.isCurrent('video') ?
      videoPageSelected : imagePageSelected
    let is = []
    for (let i = 0; i < totalPageNum; i++) {
      if (pageSelected === i + 1) {
        is.push(<li className="active" key={i + 1} onClick={this.selectPage}>{i + 1}</li>)
      } else {
        is.push(<li key={i + 1} onClick={this.selectPage}>{i + 1}</li>)
      }
    }
    let page = (
      <div className="list-page">
        <ul>
          {is}
        </ul>
      </div>
    )
    return page
  }

  onVideoCheck = e => {
    const { videoList, videoDelFiles } = this.state
    const index = videoDelFiles.indexOf(e)
    let newList
    if (index === -1) {
      newList = videoDelFiles.concat()
      newList.push(e)
    } else {
      newList = videoDelFiles.slice(0, index).concat(
        videoDelFiles.slice(index + 1)
      )
    }
    videoList.map((item, i) => {
      if(item.url == e) item.checked = !item.checked;
    })
    this.setState({
      videoDelFiles: newList,
      videoList:videoList
    })
  }

  onImageCheck = e => {
    const {imageList, imageDelFiles } = this.state
    const index = imageDelFiles.indexOf(e)
    let newList
    if (index === -1) {
      newList = imageDelFiles.concat()
      newList.push(e)
    } else {
      newList = imageDelFiles.slice(0, index).concat(
        imageDelFiles.slice(index + 1)
      )
    }
    imageList.map((item, i) => {
       if(item.url == e) item.checked = !item.checked;
    })

    this.setState({
      imageDelFiles: newList,
        imageList:imageList
    })
  }
  selectAllBtn = (e) => {
    if (this.isCurrent('video') && this.state.videoList.length) {
      const { videoList,videoDelFiles } = this.state
      let Files = [];
      let videoChecked = false
      let newList = []
      if (this.state.isVideoList) {
        videoList.map((item, i) => {
        item.checked = false
        })
      }
      else{
        videoList.map((item, i) => {
          newList.push(item.url)
          item.checked = true
        })
        videoChecked = true
        Files = newList
      }
      this.setState({
         videoList:videoList,
         videoDelFiles: Files,
         isVideoList: !this.state.isVideoList,
         videoChecked: videoChecked
      })
    }else {
       if(this.isCurrent('image') && this.state.imageList.length) {
        let imageChecked = false
        let Files = []
        let newList=[]
        const { imageList,imageDelFiles } = this.state
        if (this.state.isImageList) {
          imageList.map((item, i) => {
            item.checked = false
          })
        }
        else {
           imageList.map((item, i) => {
             newList.push(item.url)
             item.checked = true
           })
           imageChecked = true
           Files = newList
        }
        this.setState({
          imageList:imageList,
          imageDelFiles: Files,
          isImageList: !this.state.isImageList,
          imageChecked: imageChecked
        })
      }
    }
  }
  renderVideoItem () {
    const { videoList, videoItemSelected, showDeleteBtn, videoDelFiles} = this.state
    let item = null
    if (videoList.length !== 0) {
      item = videoList.map((item, i) => {
        const className = 'item' + (videoItemSelected &&
          videoItemSelected.filename === item.filename ? ' active' : '')
        return (
          <Item key={i} className={className}
            onClick={this.onVideoItemClick} url={item.url}
            onCheck={this.onVideoCheck} showDeleteBtn={showDeleteBtn}
            filename={item.filename} files={videoDelFiles} checked={item.checked}/>
        )
      })
    } else {
      item = <li className="item" style={{cursor: 'auto'}}>Video Loading...</li>
    }
    return item
  }

  renderImageItem () {
    const { imageList, imageItemSelected, showDeleteBtn, imageDelFiles } = this.state
    let item = null
    if (imageList.length !== 0) {
      item = imageList.map((item, i) => {
        const className = 'item' + (imageItemSelected &&
          imageItemSelected.filename === item.filename ? ' active' : '')
        return (
          <Item key={i} className={className}
            onClick={this.onImageItemClick} url={item.url}
            onCheck={this.onImageCheck} showDeleteBtn={showDeleteBtn}
            filename={item.filename} files={imageDelFiles} checked={item.checked}/>
        )
      })
    } else {
      item = <li className="item" style={{cursor: 'auto'}}>Image Loading...</li>
    }
    return item
  }

  render () {
    const { videoList, imageList } = this.state
    const showDelete = this.isCurrent('video') && videoList.length ||
      this.isCurrent('image') && imageList.length
    return (
      <div id="file-list">
        {showDelete ? <div className="operate">
          <div id="del_confirm" style={{ display: this.state.showDeleteBtn ? '' : 'none' }}>
            <button className="ok primary_small" onClick={this.deleteItem}>OK</button>
            <button className="cancel secondary_small" onClick={this.showDeleteBtn}>Cancel</button>
            <button className="ok secondary_small" onClick={this.selectAllBtn}>SltAll</button>
          </div>
          <button className="delete" onClick={this.showDeleteBtn}></button>
        </div> : ''}
        <div className="list-content">
          <ul id="video-list" style={{ display: this.isCurrent('video') ? '' : 'none' }}>
            {this.renderVideoItem()}
          </ul>
          <ul id="image-list" style={{ display: this.isCurrent('image') ? '' : 'none' }}>
            {this.renderImageItem()}
          </ul>
        </div>
        {this.renderPage()}
      </div>
    )
  }
}

FileList.propTypes = {
  selected: PropTypes.arrayOf(PropTypes.string.isRequired)
}

export default FileList
