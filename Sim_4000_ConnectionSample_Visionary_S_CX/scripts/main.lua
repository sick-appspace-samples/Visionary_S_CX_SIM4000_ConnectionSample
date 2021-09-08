
--Start of Global Scope---------------------------------------------------------
-- set the wanted log level - default is WARNING
Log.setLevel("INFO")
-- Variables, constants, serves etc. should be declared here.

-- create a timer to measure the received frames per second
fps_t = Timer.create()
Timer.setExpirationTime(fps_t, 10000)
Timer.setPeriodic(fps_t, true)

-- setup the 4 different viewers
local viewers =
  { View.create("v1"),
    View.create("v2"),
    View.create("v3"),
    View.create("v4") }

-- setup the 4 different remote camera receiver
local cams =
  { Image.Provider.RemoteCamera.create(),
    Image.Provider.RemoteCamera.create(),
    Image.Provider.RemoteCamera.create(),
    Image.Provider.RemoteCamera.create() }

-- configure all cameras as Visionary-S CX (V3SXX2)
cams[1]:setType("V3SXX2")
cams[2]:setType("V3SXX2")
cams[3]:setType("V3SXX2")
cams[4]:setType("V3SXX2")

-- define the IP addresses of the cameras - make sure they are to a network interface with the same subnet
cams[1]:setIPAddress("192.168.0.10")
cams[2]:setIPAddress("192.168.1.10")
cams[3]:setIPAddress("192.168.2.10")
cams[4]:setIPAddress("192.168.3.10")

-- generate point cloud converters for Planar conversion from Z image to point cloud
local pc_converters =
  { Image.PointCloudConversion.PlanarDistance.create(),
    Image.PointCloudConversion.PlanarDistance.create(),
    Image.PointCloudConversion.PlanarDistance.create(),
    Image.PointCloudConversion.PlanarDistance.create() }

-- helper function to print the camera specific parameters to the console for debugging
--@printModel(cm:Image.Provider.RemoteCamera.V3SXX2Config)
local function printModel(cm)
  local camId = cm:getCameraID()
  local width, height = cm:getCalibrationImageSize()
  local dist, unit = cm:getFocalDistance()
  local intrinsicK = cm:getIntrinsicK()
  local w2c_k1, w2c_k2, w2c_p1, w2c_p2, w2c_k3 = cm:getWorldToSensorDistortion()
  local c2w_k1, c2w_k2, c2w_p1, c2w_p2, c2w_k3 = cm:getSensorToWorldDistortion()

  Log.info(string.format("##### camera model for: %s", camId))
  Log.info(string.format("Image size %u x %u", width, height))
  Log.info(string.format("Focal distance: %.5f (%s)", dist, unit))
  Log.info(string.format("Intrinsics:\n%s", intrinsicK:toString()))
  Log.info(string.format("WordlToCamera(k1,k2,p1,p2,k3): %.5f, %.5f, %.5f, %.5f, %.5f",
    w2c_k1, w2c_k2, w2c_p1, w2c_p2, w2c_k3))
  Log.info(string.format("CameraToWorld(k1,k2,p1,p2,k3): %.5f, %.5f, %.5f, %.5f, %.5f",
    c2w_k1, c2w_k2, c2w_p1, c2w_p2, c2w_k3))
  Log.info(string.format("##### end of model"))
end

-- function to connect to a camera and initialize the point cloud converter
local function connectToCamera(cam, pc_converter)
  -- connect to device
  if cam:connect() then
    -- request config from remote camera to extract the camera model needed for the pointcloud conversion
    local cam_cfg = cam:getConfig()
    -- do not use "cam_cfg:getCameraModel()" that will not work!
    local cam_model = Image.Provider.RemoteCamera.V3SXX2Config.getCameraModel(cam_cfg)
    
    -- support also camera models of Visionary-T CX - the device type and point cloud converter needs to be changed then
    if cam_model == nil then
      Log.info("No CameraModel received from a Visionary-S CX, try if it's a Visionary-T CX.")
      cam_model = Image.Provider.RemoteCamera.V3SXX0Config.getCameraModel(cam_cfg)
    end

    -- print the data contained
    if Log.isLogable("INFO") then
      printModel(cam_model)
    end

    -- initialize the point cloud converter with the matching camera model
    pc_converter:setCameraModel(cam_model)
    -- start the image acquisition of the remote camera
    cam:start()
  else
    Log.severe('Conection to cam failed')
  end
  return cam, pc_converter
end

local function main()
  -- connect the 4 different cameras, one after the other
  for i = 1, 4, 1 do
    cams[i], pc_converters[i] = connectToCamera(cams[i], pc_converters[i])
  end
end

Script.register('Engine.OnStarted', main)

-- count the received images per camera
local imageCnt = {0,0,0,0}

local function handleNewImage(image, num)
  -- calculate the point cloud and color it with the distance values
  local pointCloud = pc_converters[num]:toPointCloud(image[1], image[1])  -- , _pixelRegion)

  -- rotate the point cloud with a mounting position (remove '--[[' and '--]]' to do so)
  --[[
  -- Values for mounting position like known from the Visionary SOPAS ET view
  -- Translation in mm
  local translationX = 0
  local translationY = 0
  local translationZ = 800
  -- Rotation in degrees calculated to radian
  local rotationX = (math.pi/180) * 0
  local rotationY = (math.pi/180) * 180
  local rotationZ = (math.pi/180) * 0

  -- create the camera-to-world transformation matrix out of it
  local cameraToWorldTransform = Transform.createRigidEuler3D("ZYX", rotationZ, rotationY, rotationX, translationX, translationY, translationZ)
  
  -- transform the existing pointcloud by replacing it
  PointCloud.transformInplace(pointCloud, cameraToWorldTransform)
  --]]

  viewers[num]:clear()
  viewers[num]:addPointCloud(pointCloud, nil)
  viewers[num]:present()

  -- function to save pointcloud to filesystem - handle carefully since space is limited
  --PointCloud.save(pointCloud, "public/cam" .. num .. "pointCloud" .. imageCnt[num] .. ".pcd", false)
  imageCnt[num] = imageCnt[num] + 1
end

--@handleOnNewImageCam1(image[+]:Image, sensordata:SensorData)
function handleOnNewImageCam1(image, sensordata)
  handleNewImage(image, 1)
end

--@handleOnNewImageCam2(image[+]:Image, sensordata:SensorData)
function handleOnNewImageCam2(image, sensordata)
  handleNewImage(image, 2)
end

--@handleOnNewImageCam3(image[+]:Image, sensordata:SensorData)
function handleOnNewImageCam3(image, sensordata)
  handleNewImage(image, 3)
end

--@handleOnNewImageCam4(image[+]:Image, sensordata:SensorData)
function handleOnNewImageCam4(image, sensordata)
  handleNewImage(image, 4)
end

-- display the received and processed framerate
function handleOnExpiredFPSCount()
  local fps = {}
  for i = 1, 4, 1 do
    fps[i] = imageCnt[i]/10
    imageCnt[i] = 0
  end
  Log.info("fps of cam #1:  " .. fps[1] .. ", #2:  " .. fps[2] .. ", #3:  " .. fps[3] .. ", #4:  " .. fps[4])
end
Timer.register(fps_t, "OnExpired", "handleOnExpiredFPSCount")
Timer.start(fps_t)

-- Event queue handling to prevent receiving queues to generate memory overflows
eventQueueHandles = {Script.Queue.create(), Script.Queue.create(), Script.Queue.create(), Script.Queue.create()}
for i = 1, 4, 1 do
  eventQueueHandles[i]:setMaxQueueSize(1)
  eventQueueHandles[i]:setFunction('handleOnNewImageCam' .. i)
  Image.Provider.RemoteCamera.register(cams[i], 'OnNewImage', 'handleOnNewImageCam' .. i)
end
