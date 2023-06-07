window.WebrtcRos = (function () {
  var newStreamId = function () {
    return (
      'webrtc_ros-stream-' + Math.floor(Math.random() * 1000000000).toString()
    )
  }

  var WebrtcRosConnection = function (signalingServerPath, configuration) {
    this.signalingServerPath =
      signalingServerPath ||
      (window.location.protocol === 'https:' ? 'wss://' : 'ws://') +
        window.location.host +
        '/webrtc'
    this.onConfigurationNeeded = undefined
    this.signalingChannel = null
    this.peerConnection = null
    this.receiveChannel = null
    this.sendChannel = null
    this.dataChannelActive = false

    this.receiveBuffer = []
    // this.receivedSize = 0
    this.fileReader = null
    this.receivedDataBuffer = new Uint8Array()
    this.receivedDataChunks = []
    this.peerConnectionMediaConstraints = {
      optional: [{ DtlsSrtpKeyAgreement: true }]
    }
    this.peerConnectionConfiguration = configuration

    this.lastConfigureActionPromise = Promise.resolve([])

    this.addStreamCallbacks = {}
    this.removeTrackCallbacks = {}
  }

  WebrtcRosConnection.prototype.sendMessage = function (data) {
    console.log('ROS2 server has been launched: ', data)
    var message = data
    this.sendChannel.send(message)
  }

  WebrtcRosConnection.prototype.handleFileInputChange = async function () {
    const file = this.fileInput.files[0]
    if (!file) {
      console.log('No file chosen')
    } else {
      this.sendFileButton.disabled = false
    }
  }

  WebrtcRosConnection.prototype.connect = function () {
    var self = this
    this.close()
    this.signalingChannel = new WebSocket(this.signalingServerPath)
    this.signalingChannel.onmessage = function (e) {
      self.onSignalingMessage(e)
    }
    this.signalingChannel.onopen = function () {
      console.log('WebRTC signaling connection established')
      if (self.onConfigurationNeeded) {
        self.onConfigurationNeeded()
      }
    }
    this.signalingChannel.onerror = function () {
      console.error('WebRTC signaling error')
    }
    this.signalingChannel.onclose = function () {
      console.log('WebRTC signaling connection closed')
    }

    this.peerConnection = new RTCPeerConnection(
      this.peerConnectionConfiguration,
      this.peerConnectionMediaConstraints
    )

    var dataChannelOptions = {
      ordered: true,
      maxPacketLifeTime: 0
    }
    this.sendChannel = this.peerConnection.createDataChannel(
      'data_channel_2',
      dataChannelOptions
    )
    this.sendChannel.bufferedAmountLowThreshold = 1024 * 1024 * 16 // 1 MB
    // this.sendChannel.binaryType = 'arraybuffer'
    this.sendChannel.onopen = function (event) {
      console.log('*** Channel Send Funcion Has Opened ***')
    }
    this.sendChannel.onclose = function (event) {
      console.log('*** Channel Send Funcion Closed ***')
    }
    this.sendChannel.onerror = function (event) {
      console.log(event.error)
    }

    // Set up the Three.js scene
    // const loader = new THREE.PCDLoader()
    const scene = new THREE.Scene()
    scene.rotation.z = Math.PI/2;
    const camera = new THREE.PerspectiveCamera(
      45,
      window.innerWidth / window.innerHeight,
      0.1,
      40
    )
    camera.position.set(0, 0, 4);
    scene.add(camera);
    const renderer = new THREE.WebGLRenderer()
    renderer.setPixelRatio(window.devicePixelRatio)
    renderer.setSize(window.innerWidth, window.innerHeight)
    document.body.appendChild(renderer.domElement)

    // tone mapping
    renderer.toneMapping = THREE.NoToneMapping

    renderer.outputEncoding = THREE.sRGBEncoding

    // Create a material and geometry for the cubes
    const material = new THREE.MeshBasicMaterial({ color: 0x00ff00 })
    const geometry = new THREE.BoxGeometry(0.008, 0.008, 0.008)
    // const geometry = new THREE.SphereGeometry(0.005, 16, 16)
    
    //const controls = new OrbitControls(camera, renderer.domElement);
    //            controls.addEventListener('change',  renderer); // use if there is no animation loop
     //           controls.minDistance = 0.5;
    //            controls.maxDistance = 10;
    let receivedSize = 0
    let receivedData = "";
    
    this.sendChannel.addEventListener('message', event => {
    	if (event.data == "END") {
        try {

          const json_obj = JSON.parse(receivedData)
          for (const point of json_obj) {
            //const x = point.x
            //const y = point.y
            //const z = point.z
            //var temp = point.r
            
            //var temp1 = point.x
            //const x = ((point.x >> 16) & 0xff)
            //const y = ((point.x >> 8) & 0xff)
            //const z = (point.x & 0xff)
            //const b = (((point.r >> 16) & 0xff) / 255)
            //const g = (((point.r >> 8) & 0xff) / 255)
            //const r = ((point.r & 0xff) / 255)
            //console.log(`Received ${(x-200) / 100} ,${(y-200) / 100},${(z-200) / 100}`)
            const color = new THREE.Color((((point.r >> 16) & 0xff) / 255), (((point.r >> 8) & 0xff) / 255), ((point.r & 0xff) / 255))
            const material = new THREE.MeshBasicMaterial({ color: color })
            const cube = new THREE.Mesh(geometry, material)
            cube.position.set((((point.x >> 16) & 0xff)-100) / 100, (((point.x >> 8) & 0xff)-100) / 100, ((point.x & 0xff)-100) / 100)
            scene.add(cube)
          }
          renderer.render(scene, camera)
          for (const cube of scene.children) {
            scene.remove(cube)
          }
          
          
        } catch (error) {
          console.error('Failed to parse JSON data: ', error)
        }


        receivedData = "";
      } else {
        receivedData += event.data;
      }
      

    })

    this.peerConnection.ondatachannel = function (event) {
      this.receiveChannel = event.channel
      // this.receiveChannel.onmessage = function(event) {
      // 	console.log("*** receive: ",event.data);
      // };
      this.receiveChannel.onopen = function (event) {
        console.log('*** Channel Receive Funcion Has Opened ***')
      }
      this.receiveChannel.onclose = function (event) {
        console.log('*** Channel Received Funcion Closed ***')
      }
      this.receiveChannel.onerror = function (err) {
        console.log(err)
      }
      this.receiveChannel.addEventListener('message', event => {
        console.log('*** receive: ', event.data)
      })
    }

    this.peerConnection.onicecandidate = function (event) {
      if (event.candidate) {
        var candidate = {
          sdp_mline_index: event.candidate.sdpMLineIndex,
          sdp_mid: event.candidate.sdpMid,
          candidate: event.candidate.candidate,
          type: 'ice_candidate'
        }
        self.signalingChannel.send(JSON.stringify(candidate))
      }
    }

    this.peerConnection.ontrack = function (event) {
      var callbackData = self.addStreamCallbacks[event.streams[0].id]
      if (callbackData) {
        event.streams[0].onremovetrack = function (event) {
          var callbackData = self.removeTrackCallbacks[event.track.id]
          if (callbackData) {
            callbackData.resolve({
              track: event.track
            })
          }
        }
        callbackData.resolve({
          stream: event.streams[0],
          remove: new Promise(function (resolve, reject) {
            self.removeTrackCallbacks[event.track.id] = {
              resolve: resolve,
              reject: reject
            }
          })
        })
      }
    }
  }
  WebrtcRosConnection.prototype.close = function () {
    if (this.peerConnection) {
      this.peerConnection.close()
      this.peerConnection = null
    }
    if (this.signalingChannel) {
      this.signalingChannel.close()
      this.signalingChannel = null
    }
    if (this.sendChannel) {
      this.sendChannel.onopen = null
      this.sendChannel.onclose = null
      this.sendChannel.close()
      this.sendChannel = null
    }
    if (this.receiveChannel) {
      this.receiveChannel.onmessage = null
      this.receiveChannel.onopen = null
      this.receiveChannel.onclose = null
      this.receiveChannel.close()
      this.receiveChannel = null
    }
  }
  WebrtcRosConnection.prototype.onSignalingMessage = function (e) {
    var self = this
    var dataJson = JSON.parse(e.data)
    if (dataJson.type === 'offer') {
      console.log('Received WebRTC offer via WebRTC signaling channel')
      this.peerConnection.setRemoteDescription(
        new RTCSessionDescription(dataJson),
        function () {
          self.sendAnswer()
        },
        function (event) {
          console.error('onRemoteSdpError', event)
        }
      )
    } else if (dataJson.type === 'ice_candidate') {
      console.log('Received WebRTC ice_candidate via WebRTC signaling channel')
      var candidate = new RTCIceCandidate({
        sdpMLineIndex: dataJson.sdp_mline_index,
        candidate: dataJson.candidate
      })
      this.peerConnection.addIceCandidate(candidate)
    } else {
      console.warn(
        "Received unknown message type '" +
          dataJson.type +
          "' via WebRTC signaling channel"
      )
    }
  }

  WebrtcRosConnection.prototype.sendAnswer = function () {
    var self = this
    var mediaConstraints = { optional: [{ OfferToReceiveVideo: true }] }
    this.peerConnection.createAnswer(
      function (sessionDescription) {
        self.peerConnection.setLocalDescription(sessionDescription)
        var data = JSON.stringify(sessionDescription)
        self.signalingChannel.send(data)
      },
      function (error) {
        console.warn('Create answer error:', error)
      },
      mediaConstraints
    )
  }
  WebrtcRosConnection.prototype.addRemoteStream = function (config) {
    var stream_id = newStreamId()
    var self = this

    this.lastConfigureActionPromise = this.lastConfigureActionPromise.then(
      function (actions) {
        actions.push({ type: 'add_stream', id: stream_id })
        if (config.video) {
          actions.push({
            type: 'add_video_track',
            stream_id: stream_id,
            id: stream_id + '/' + config.video.id,
            src: config.video.src
          })
        }
        if (config.audio) {
          actions.push({
            type: 'add_audio_track',
            stream_id: stream_id,
            id: stream_id + '/' + config.audio.id,
            src: config.audio.src
          })
        }
        return actions
      }
    )
    return new Promise(function (resolve, reject) {
      self.addStreamCallbacks[stream_id] = {
        resolve: resolve,
        reject: reject
      }
    })
  }
  WebrtcRosConnection.prototype.addRemoteStream1 = function (config) {
    var stream_id = newStreamId()
    var self = this

    this.lastConfigureActionPromise = this.lastConfigureActionPromise.then(
      function (actions) {
        actions.push({ type: 'add_stream', id: stream_id })
        if (config.video) {
          actions.push({
            type: 'add_video_track',
            stream_id: stream_id,
            id: stream_id + '/' + config.video.id,
            src: config.video.src
          })
        }
        if (config.audio) {
          actions.push({
            type: 'add_audio_track',
            stream_id: stream_id,
            id: stream_id + '/' + config.audio.id,
            src: config.audio.src
          })
        }
        return actions
      }
    )
    return new Promise(function (resolve, reject) {
      self.addStreamCallbacks[stream_id] = {
        resolve: resolve,
        reject: reject
      }
    })
  }
  WebrtcRosConnection.prototype.removeRemoteStream = function (stream) {
    var self = this
    this.lastConfigureActionPromise = this.lastConfigureActionPromise.then(
      function (actions) {
        actions.push({ type: 'remove_stream', id: stream.id })
        return actions
      }
    )
  }
  WebrtcRosConnection.prototype.addLocalStream = function (
    user_media_config,
    local_stream_config
  ) {
    var self = this
    return new Promise(function (resolve, reject) {
      self.lastConfigureActionPromise = self.lastConfigureActionPromise.then(
        function (actions) {
          return navigator.mediaDevices
            .getUserMedia(user_media_config)
            .then(function (stream) {
              actions.push({ type: 'expect_stream', id: stream.id })
              if (local_stream_config.video) {
                actions.push({
                  type: 'expect_video_track',
                  stream_id: stream.id,
                  id: stream.getVideoTracks()[0].id,
                  dest: local_stream_config.video.dest
                })
              }
              self.peerConnection.addStream(stream)
              resolve({
                stream: stream,
                remove: new Promise(function (resolve, reject) {
                  self.removeStreamCallbacks[stream.id] = {
                    resolve: resolve,
                    reject: reject
                  }
                })
              })
              return actions
            })
        }
      )
    })
  }
  WebrtcRosConnection.prototype.removeLocalStream = function (stream) {
    var self = this
    this.lastConfigureActionPromise = this.lastConfigureActionPromise.then(
      function (actions) {
        console.log('Removing stream')
        self.peerConnection.removeStream(stream)
        var callbackData = self.removeStreamCallbacks[stream.id]
        if (callbackData) {
          callbackData.resolve({
            stream: stream
          })
        }
        return actions
      }
    )
  }
  WebrtcRosConnection.prototype.sendConfigure = function () {
    var self = this
    var currentLastConfigureActionPromise = this.lastConfigureActionPromise
    this.lastConfigureActionPromise = Promise.resolve([])

    currentLastConfigureActionPromise.then(function (actions) {
      var configMessage = { type: 'configure', actions: actions }
      self.signalingChannel.send(JSON.stringify(configMessage))
      console.log('WebRTC ROS Configure: ', actions)
    })
  }

  var WebrtcRos = {
    createConnection: function (signalingServerPath, configuration) {
      return new WebrtcRosConnection(signalingServerPath, configuration)
    }
  }
  return WebrtcRos
})()
