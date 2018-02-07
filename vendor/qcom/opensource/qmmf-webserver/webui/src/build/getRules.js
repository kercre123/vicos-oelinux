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

const config = require('../config')
const path = require('path')
const __DEV__ = config.global.__DEV__
const ExtractTextPlugin = require('extract-text-webpack-plugin')

const assetsPath = function (_path) {
  var assetsSubDirectory = 'static'
  return path.posix.join(assetsSubDirectory, _path)
}

module.exports = function () {
  const rules = []

  rules.push({
    test: /\.jsx?$/,
    exclude: /node_modules/,
    loaders: [
      { loader: 'babel-loader' }
    ]
  })

  rules.push({
    test: /\.js$/,
    include: eval("/node_modules\\"+path.sep+"(html5[_A-Za-z]*\\"+path.sep+"|bp[_A-Za-z]*\\"+path.sep+"|va[_A-Za-z]*\\"+path.sep+")/"),
    loaders: [
      { loader: 'babel-loader',
        query: {
          presets: ['es2015', 'stage-3', 'stage-2', 'stage-1', 'stage-0']
        }
      }
    ]
  })
 
  if (__DEV__) {
  
    rules.push({
      test: /\.(scss|sass)$/i,
      include: /src/,
      loaders: [
        'style-loader',
        { loader: 'css-loader', options: { importLoaders: 1} },
        { loader: 'postcss-loader' },
        { loader: 'sass-loader' }
      ]
    })

    rules.push({
      test: /\.css$/,
      loaders: [
        'style-loader',
        { loader: 'css-loader', options: { importLoaders: 1 } }
      ]
    })
  } else {
 
    rules.push({
      test: /\.(scss|sass)$/i,
      include: /src/,
      loader: ExtractTextPlugin.extract({
        fallback: "style-loader",
        use: [
          "css-loader?minimize&importLoaders=1",
          "postcss-loader",
          "sass-loader"
        ],
        publicPath: '../../'
      })
    })
 
    rules.push({
      test: /\.css$/,
      include: /src|node_modules/,
      loader: ExtractTextPlugin.extract({
        fallback: "style-loader",
        use: "css-loader?minimize&importLoaders=1",
        publicPath: '../../'
      })
    })
  }
 
  rules.push({
    test: /\.(png|gif|svg)(\?.*)?$/,
    loader: 'url-loader',
    query: {
      limit: 20000,
      name: assetsPath('img/[name].[ext]')
    }
  })

  rules.push({
    test: /\.(jpg|jpeg)(\?.*)?$/,
    loader: 'url-loader',
    query: {
      limit: 20000,
      name: assetsPath('img/[name].[ext]')
    }
  })

  return rules
}
