function sysCall_init()

    -- Get vision sensor handles
    vs_left = sim.getObjectHandle('Vision_sensor_left')
    vs_right = sim.getObjectHandle('Vision_sensor_right')

    -- Enable image publishers for left and right sensors + depth outputs
    if simROS then
        sim.addLog(sim.verbosity_scriptinfos,"ROS interface was found.")
        pub_left=simROS.advertise('/image/left/rgb', 'sensor_msgs/Image')
        simROS.publisherTreatUInt8ArrayAsString(pub_left)
        -- sub_left=simROS.subscribe('/image/left', 'sensor_msgs/Image', 'imageMessage_callback')
        -- simROS.subscriberTreatUInt8ArrayAsString(sub_left)

        pub_right=simROS.advertise('/image/right/rgb', 'sensor_msgs/Image')
        simROS.publisherTreatUInt8ArrayAsString(pub_right)
        -- sub_right=simROS.subscribe('/image/right', 'sensor_msgs/Image', 'imageMessage_callback')
        -- simROS.subscriberTreatUInt8ArrayAsString(sub_right)

        pub_left_d = simROS.advertise('/image/left/depth', 'sensor_msgs/Image')
        simROS.publisherTreatUInt8ArrayAsString(pub_left_d)
        -- print("DBM: ", sim.handleflag_depthbuffermeters)
        -- print("Coded String: ", sim.handleflag_codedstring)
        pub_right_d = simROS.advertise('/image/right/depth', 'sensor_msgs/Image')
        simROS.publisherTreatUInt8ArrayAsString(pub_right_d)
    else
        sim.addLog(sim.verbosity_scripterrors,"ROS interface was not found. Cannot run.")
    end
end

function sysCall_actuation()
    -- put your actuation code here
end

function sysCall_sensing()
  if simROS then
      -- Publish the image of the left vision sensor:
      local data_left,w_left,h_left=sim.getVisionSensorCharImage(vs_left)
      d={}
      d.header={stamp=simROS.getTime(), frame_id='a'}
      d.height=h_left
      d.width=w_left
      d.encoding='rgb8'
      d.is_bigendian=1
      d.step=w_left*3
      d.data=data_left

      simROS.publish(pub_left,d)

      -- Publish the image of the right vision sensor:
      local data_right,w_right,h_right=sim.getVisionSensorCharImage(vs_right)
      d={}
      d.header={stamp=simROS.getTime(), frame_id='a'}
      d.height=h_right
      d.width=w_right
      d.encoding='rgb8'
      d.is_bigendian=1
      d.step=w_right*3
      d.data=data_right
      simROS.publish(pub_right,d)


      -- Publish the depth buffer of the left vision sensor
      local data_left_d = sim.getVisionSensorDepthBuffer(vs_left+sim.handleflag_codedstring+sim.handleflag_depthbuffermeters)
      local res,nearClip = sim.getObjectFloatParameter(vs_left, sim.visionfloatparam_near_clipping) -- near clipping plane - leave in metres
      local res,farClip = sim.getObjectFloatParameter(vs_left, sim.visionfloatparam_far_clipping) -- near clipping plane - leave in metres
      data_left_d = sim.transformBuffer(data_left_d, sim.buffer_float, farClip-nearClip, nearClip, sim.buffer_float)
      local res=sim.getVisionSensorResolution(vs_left)
      w_left_d, h_left_d = res[1], res[2]
      dd={}
      dd.header={stamp=simROS.getTime(), frame_id='a'}
      dd.width=w_left_d
      dd.height=h_left_d
      dd.encoding='32FC1'
      dd.is_bigendian=0
      dd.step=w_left_d*2
      dd.data = data_left_d

      simROS.publish(pub_left_d, dd)

      -- Publish the depth buffer of the right vision sensor
      local data_right_d = sim.getVisionSensorDepthBuffer(vs_right+sim.handleflag_codedstring)
      local res,nearClip = sim.getObjectFloatParameter(vs_right, sim.visionfloatparam_near_clipping) -- near clipping plane - leave in metres
      local res,farClip = sim.getObjectFloatParameter(vs_right, sim.visionfloatparam_far_clipping) -- near clipping plane - leave in metres
      data_right_d = sim.transformBuffer(data_right_d, sim.buffer_float, farClip-nearClip, nearClip, sim.buffer_uint16)
      local res=sim.getVisionSensorResolution(vs_right)
      w_right_d, h_right_d = res[1], res[2]
      dd={}
      dd.header={stamp=simROS.getTime(), frame_id='a'}
      dd.width=w_right_d
      dd.height=h_right_d
      dd.encoding='16UC1'
      dd.is_bigendian=1
      dd.step=w_right_d*2
      dd.data = data_right_d

      simROS.publish(pub_right_d, dd)
  end
end

function sysCall_cleanup()
    -- Shut down left camera publisher and subscriber
    simROS.shutdownPublisher(pub_left)
    simROS.shutdownPublisher(pub_left_d)
    -- simROS.shutdownSubscriber(sub_left)
    -- Shut down right camera publisher and subscriber
    simROS.shutdownPublisher(pub_right)
    simROS.shutdownPublisher(pub_right_d)
    -- simROS.shutdownSubscriber(sub_right)
end

function imageMessage_callback(msg)
    -- Apply the received image to the passive vision sensor that acts as an image container
    -- sim.setVisionSensorCharImage(passiveVisionSensor,msg.data)
end
