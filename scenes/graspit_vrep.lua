--Description: This is a plugin for Graspit! and V-rep, so that grasp information form Graspit! can be passed
--into V-rep and use these grasps to pick up the object.


function csvfile_read(path, sep, tonum)
    tonum = tonum or true
    sep = sep or ','
    local csvFile = {}
    local file = assert(io.open(path, "r"))
    for line in file:lines() do
        fields = csvfile_split(line, sep)
        if tonum then -- convert numeric fields to numbers
            for i=1,#fields do
                fields[i] = tonumber(fields[i]) or fields[i]
            end
        end
        table.insert(csvFile, fields)
    end
    file:close()
    return csvFile
end

function csvfile_read_idx(path, sep, tonum, idx_low, idx_high)
    tonum = tonum or true
    sep = sep or ','
    local n = 0
    idx_low = tonumber(idx_low)
    idx_high = tonumber(idx_high)
    
    local csvFile = {}
    
    local file = assert(io.open(path, "r"))
    for line in file:lines() do
        n = n + 1

        if n >= idx_low and n <idx_high then
            fields = csvfile_split(line, sep)
            if tonum then -- convert numeric fields to numbers
                for i=1,#fields do
                    fields[i] = tonumber(fields[i]) or fields[i]
                end
            end
            table.insert(csvFile, fields)
        end
        if n >= idx_high then
            break
        end
    end
    file:close()
    return csvFile
end

--This is how they format their reading data from csv file
--First, check what information we need and format our data in similar way
--Also, check how they use their formatted data
function parse_data(data)

    local obj_names, grasp_nums, obj_mats = {}, {}, {}
    local angles, gripper_mats, obj_com, obj_inertia = {}, {}, {}, {}

    for i = 1, #data, 1 do
    
        local obj_name = data[i][1]
        
        local grasp_num = data[i][2]
        
        -- 3x4 object matrix from initial pose collection scene
        local obj_mat = {
            data[i][3], data[i][4], data[i][5], data[i][6],
            data[i][7], data[i][8], data[i][9], data[i][10],
            data[i][11], data[i][12], data[i][13], data[i][14]}
            
        -- Local and global rotations
        local angle = {
            data[i][15], data[i][16], data[i][17], 
            data[i][18], data[i][19], data[i][20]}
            
        -- 3x4 gripper matrix from grasp candidate generation
        local gripper_mat = {
            data[i][21], data[i][22], data[i][23], data[i][24],
            data[i][25], data[i][26], data[i][27], data[i][28],
            data[i][29], data[i][30], data[i][31], data[i][32]}     
            
        -- x,y,z location of center of mass
        local com = {data[i][33], data[i][34], data[i][35]}
        
        -- 3x3 inertia matrix
        local inertia = {
            data[i][36], data[i][37], data[i][38],
            data[i][39], data[i][40], data[i][41],
            data[i][42], data[i][43], data[i][44]}

        table.insert(obj_names, obj_name)
        table.insert(grasp_nums, grasp_num)
        table.insert(obj_mats, obj_mat)
        table.insert(angles, angle)
        table.insert(gripper_mats, gripper_mat)
        table.insert(obj_com, com)
        table.insert(obj_inertia, inertia)
    end
    
    local data = {}
    data['obj_names'] = obj_names
    data['grasp_nums'] = grasp_nums
    data['obj_mats'] = obj_mats
    data['angles'] = angles
    data['gripper_mats'] = gripper_mats
    data['obj_com'] = obj_com
    data['obj_inertia'] = obj_inertia
    return data 
end



-- MAIN FUNCTION
threadFunction=function()

    simResetDynamicObject(h_object)
    simSetModelProperty(h_gripper_object, GRIPPER_PROP_VISIBLE)
    simSwitchThread()

    attempt = 0
    local grasp_number = 0
    local object_initial_matrix = simGetObjectMatrix(h_object, h_workspace)
    
    -- Loop through each of our grasp candidates
    for i = 1, #gl_grasp_candidates, 1 do
        
        attempt = attempt + 1
    
        -- Reset the object for each grasp attempt
        simSetModelProperty(h_gripper_object, GRIPPER_PROP_INVISIBLE)
        simSetObjectMatrix(h_object, h_workspace, object_initial_matrix)
        
        -- Going to randomly set a finger spread angle (for Barrett Hand)
        local seed = math.random(#GRIPPER_FINGER_ANGLES)
        local finger_angle = GRIPPER_FINGER_ANGLES[seed]
        simSetScriptSimulationParameter(sim_handle_all, 'fingerAngle', finger_angle)
        simSwitchThread()
        
        -- Set the proximity sensor (on the hand) to be candidate location
        sampling_matrix = gl_grasp_pose[i]
        simSetObjectMatrix(h_gripper_proxim, h_object, sampling_matrix)
        simSwitchThread()

        -- Check if gripper is pointing towards object & is not too low
        local r, dist, point = simCheckProximitySensor(h_gripper_proxim, h_object)
        simSwitchThread()
        
        if r == 1 then 

            -- The direction vector is the x,y,z components of a line eminating
            --   from the gripper to some given point on the object. We're
            --   going to move the gripper along this line (closer to the 
            --   object), so we need to normalize it first. 
            local dvec = {-point[1], -point[2], -point[3]}
            local mag = math.sqrt( dvec[1]^2 + dvec[2]^2 + dvec[3]^2)
            dvec = {dvec[1]/mag, dvec[2]/mag, dvec[3]/mag}

            -- Going to record the initial position of the hand, so we can try
            --   grasping from a few different distances to the object
            local prox_mtx = simGetObjectMatrix(h_gripper_proxim, h_workspace)
            local sensor_wrt_object = simGetObjectMatrix(h_gripper_proxim, h_object)
            point_wrt_object = simMultiplyVector(sensor_wrt_object, point)
            simSwitchThread()
            
            -- Display where the detected point was
            local proxim_wrt_world = simGetObjectMatrix(h_gripper_proxim, -1)
            local point_wrt_world = simMultiplyVector(proxim_wrt_world, point)
            simAddDrawingObjectItem(sphereContainer, point_wrt_world)

            -- Try grasping object from different distances along the approach
            for contact_dist = 1, #gl_palm_dist, 1 do

                -- ----------------- POSITION THE GRIPPER --------------------

                -- For each grasp attempted, we need to reset the configuration
                simSetConfigurationTree(GRIPPER_CONFIGURATION)
                simSwitchThread()
                
                -- Reset position of contact dummy
                simSetObjectMatrix(h_gripper_dummy, h_workspace, prox_mtx)
                simSwitchThread()
                
                -- Set the gripper to be at the detected object surface point 
                simSetObjectPosition(h_gripper_dummy, h_gripper_dummy, point)
                simSwitchThread()

                local approach = {
                    dvec[1]*gl_palm_dist[contact_dist],
                    dvec[2]*gl_palm_dist[contact_dist],
                    dvec[3]*gl_palm_dist[contact_dist]}
                    
                -- Move the hand backwards a distance 'd' from the surface
                simSetObjectPosition(h_gripper_dummy, h_gripper_dummy, approach)
                simSwitchThread()

                -- While moving the hand, we need to deal with moving a dynamic
                --   object, so wedon't introduce any instantaneous 
                --   accelerations, or introduce errors for the dynamics engine
                simSetModelProperty(h_gripper_object, GRIPPER_PROP_STATIC)
                simSwitchThread()
                moveGripper(h_gripper_object, h_workspace, h_gripper_dummy)
                simSwitchThread()
                simSetModelProperty(h_gripper_object, GRIPPER_PROP_VISIBLE)
                simSwitchThread()
                
                -- -- ----------------- COLLECT CAMERA IMAGES -------------------
                
                -- -- Randomly set the colour of the object
                -- local randr = math.random()
                -- local randg = math.random()
                -- local randb = math.random()
                -- local colour = {randr, randg, randb}
                -- simSetShapeColor(
                --     h_object, nil, sim_colorcomponent_ambient_diffuse, colour)

                -- -- For positioning the camera, we're going to do something 
                -- --   similar to what we did for positioning the manipulator. 
                -- --   We start by setting the camera location equal to the grasp
                -- --   location, then moving it backwards by a given offset
                -- local contact_matrix = simGetObjectMatrix(h_gripper_dummy, h_workspace)
                -- simSetObjectMatrix(h_camera_dummy, h_workspace, contact_matrix)
                -- simSwitchThread()

                -- local camera_approach = 
                --     {dvec[1]*myconf.camera_contact_offset,
                --      dvec[2]*myconf.camera_contact_offset,
                --      dvec[3]*myconf.camera_contact_offset}
                -- simSetObjectPosition(h_camera_dummy, h_camera_dummy, camera_approach)
                -- simSwitchThread()

                -- -- Once we've positioned the camera, we need to make sure that 
                -- --   it points towards the detected surface point, while the 
                -- --   y-axis points upwards
                -- pointObjectTowardsLocation(h_camera_dummy, point_wrt_world, {0,0,1})
                -- simSwitchThread()



                -- -- Collect one camera image where we're looking at the object, but rotation
                -- -- invariant
                -- local cm = simGetObjectMatrix(h_camera_dummy, h_workspace)
                -- simSetObjectMatrix(h_camera_depth, h_workspace, cm)
                -- simSetObjectMatrix(h_camera_colour, h_workspace, cm)
                -- simSetObjectMatrix(h_camera_mask, h_workspace, cm)
                -- simSwitchThread()

                -- simHandleVisionSensor(h_camera_depth)
                -- simHandleVisionSensor(h_camera_colour)
                -- simHandleVisionSensor(h_camera_mask)
                -- simSwitchThread()

                -- local object_image = simGetVisionSensorDepthBuffer(h_camera_depth)
                -- local colour_image = simGetVisionSensorImage(h_camera_colour)
                -- local mask_image = simGetVisionSensorImage(h_camera_mask)
                -- rot_invariant_matrix = simGetObjectMatrix(h_camera_dummy, h_workspace)
                -- simSwitchThread()
                -- simWait(5)


                -- -- Collect one camera image where we're at the same orientation as the gripper
                -- local orient = simGetObjectOrientation(h_gripper_dummy, -1)
                -- simSetObjectOrientation(h_camera_dummy, -1, orient)

                -- local cm = simGetObjectMatrix(h_camera_dummy, h_workspace)
                -- simSetObjectMatrix(h_camera_depth, h_workspace, cm)
                -- simSetObjectMatrix(h_camera_colour, h_workspace, cm)
                -- simSwitchThread()

                -- simHandleVisionSensor(h_camera_depth)
                -- simHandleVisionSensor(h_camera_colour)
                -- simSwitchThread()

                -- local direct_depth_image = simGetVisionSensorDepthBuffer(h_camera_depth)
                -- local direct_colour_image = simGetVisionSensorImage(h_camera_colour)
                -- rot_variant_matrix = simGetObjectMatrix(h_camera_dummy, h_workspace)
                -- simSwitchThread()
                -- simWait(5)


                -- -- Visualizing the camera view direction with a line
                -- local cam_pt = simGetObjectPosition(h_camera_dummy, -1)
                -- simAddDrawingObjectItem(
                --     lineContainer, {point_wrt_world[1], point_wrt_world[2], 
                --     point_wrt_world[3], cam_pt[1], cam_pt[2], cam_pt[3]})


                -- -------------- ATTEMPT TO GRASP OBJECT --------------------
                                    
                -- Check that the gripper is curretly not contacting anything, 
                -- and that the camera is above the workspace (table) level
                is_no_contact = verifyContacts()
                local camera_wrt_workspace = simGetObjectPosition(h_camera_dummy, h_table_object)
                simSwitchThread()
                
                if is_no_contact == true then 
                
                    -- Clear the previous grasp configuration
                    simSetIntegerSignal('clearGrasp',1)
                    simClearIntegerSignal('grasp_done')
                    simSwitchThread()

                    -- Send a signal to close the gripper
                    simSetIntegerSignal('closeGrasp',1)
                    simWaitForSignal('grasp_done')
                    simSwitchThread()
                    simClearIntegerSignal('grasp_done')
                    simSwitchThread()
                    simWait(3)

                    -- Send a signal to hold the grasp, while we lift the obj
                    simClearIntegerSignal('closeGrasp')
                    simSetIntegerSignal('holdGrasp',1)
                    simWaitForSignal('grasp_done')
                    simSwitchThread()

                    -- Check that all gripper fingertips are touching object
                    local r, contactsObject = checkContacts(h_gripper_contacts)
                    simSwitchThread()

                    -- ---------------- LIFTING OBJECT -----------------------
                    if r ~= nil then
                        
                        -- -- Save the Pre-grasp data
                        -- local pregrasp_info = getGraspInformation(
                        --     r, contactsObject, GRIPPER_COMPONENTS, h_workspace)
                        -- simSwitchThread()

                        -- Make object dynamic
                        simSetObjectInt32Parameter(
                            h_object, sim_shapeintparam_static, 0) 

                        -- Perform a non-blocking object movement)
                        local initialP = simGetObjectPosition(h_gripper_object, -1)
                        local targetP = simGetObjectPosition(h_workspace, -1)
                        targetP = {targetP[1], targetP[2], targetP[3]+0.5}

                        local targetPosVel = 
                            {targetP[1], targetP[2], targetP[3], 0, 0, 0}
                        
                        local maxVelAccelJerk =
                            {gl_vel[1],  gl_vel[2],  gl_vel[3], 
                             gl_accel[1],gl_accel[2],gl_accel[3], 
                             gl_jerk[1], gl_jerk[2], gl_jerk[3]}
                                                 
                        local posVelAccel = 
                            {initialP[1], initialP[2], initialP[3], 0, 0, 0, 
                            0, 0, 0, 0}
                        
                        local rmlHandle = simRMLPos(
                            3, 0.001, -1, posVelAccel, maxVelAccelJerk, 
                            {1,1,1}, targetPosVel)
                            
                        -- To incrementally move the hand along the generated 
                        --   path, we need to do a little movement at each time 
                        --   step.
                        print('lifting object ... ')
                        local res = 0
                        while res==0 do
                            dt=simGetSimulationTimeStep()
                            res,posVelAccel,sync=simRMLStep(rmlHandle,dt)
                            simSetObjectPosition(h_gripper_object, -1, posVelAccel)
                            simSwitchThread()
                        end
                        simRMLRemove(rmlHandle)
                        simSwitchThread()

                        -- Check whether the fingers are still in contact 
                        --   with the object
                        local r, contactsLift = checkContacts(h_gripper_contacts)

                        -- Check if the object is in contact with table, if so
                        --   then the grasp is a fail.
                        local r2 = simCheckCollision(h_object, h_table_object)
                        if r2 == 1 then r = 0 end

                        if r == 0 then
                            print('Grasp attempt: '..attempt..' failure!')
                        else
                            print('Grasp attempt: '..attempt..' success!')
                        end

                        if r == 1 then 

                        -- Get postgrasp information
                        local info = getGraspInformation(
                            r, contactsLift, GRIPPER_COMPONENTS, h_workspace)
                        simSwitchThread()

                        -- ---------------- SAVE THE DATA --------------------
                        -- csvfile_write(savefile, table.copy({"DIRECT_DEPTH"}, direct_depth_image))
                        -- simSwitchThread()
                        -- csvfile_write(savefile, table.copy({"DIRECT_COLOUR"}, direct_colour_image))
                        -- simSwitchThread()

                        -- csvfile_write(savefile, 
                        --     table.copy({"GRIPPER_IMAGE"}, object_image))
                        -- simSwitchThread()
                        
                        -- csvfile_write(savefile, 
                        --     table.copy({"GRIPPER_IMAGE_COLOUR"}, colour_image))
                        -- simSwitchThread()
                        
                        -- csvfile_write(savefile, 
                        --     table.copy({"GRIPPER_MASK_IMAGE"}, mask_image))
                        -- simSwitchThread()
                        
                        -- csvfile_write(savefile, 
                        --     table.copy({"GRIPPER_HEADER"}, pregrasp_info[1]))
                        -- simSwitchThread()
                        
                        -- csvfile_write(savefile, 
                        --     table.copy({"GRIPPER_PREGRASP"}, pregrasp_info[2]))
                        -- simSwitchThread()
                        
                        -- csvfile_write(savefile, 
                        --     table.copy({"GRIPPER_POSTGRASP"}, info[2]))
                        -- simSwitchThread()
                        simWait(5)

                        end
                        
                        -- Finish the grasp attempt by making the object
                        --   static again, and reseting to the initial position
                        simSetObjectInt32Parameter(h_object, sim_shapeintparam_static, 1)
                        simResetDynamicObject(h_object)
                        simSetObjectMatrix(h_object, h_workspace, object_initial_matrix)

                    end -- Lifting object
                    
                end -- Grasping object
            
            end -- Iterating over distances away from object surface
        
        end -- If gripper is pointing towards the object
        
        
        if math.mod(attempt, 50) == 0 then
            local n = #gl_grasp_candidates
            print('--------------------------------------------------------')
            print(attempt..'/'.. n ..' successful '..gl_obj_names[1] ) 
            print('--------------------------------------------------------')
        end
        
        -- Free some memory?
        simAddDrawingObjectItem(sphereContainer,nil) 
        simAddDrawingObjectItem(lineContainer,nil) 

        contactsLift = nil
        info = nil
        object_image = nil
        colour_image = nil
        mask_image = nil
        pregrasp_info = nil
        contactsObject = nil
        
    end -- loop over all rotation combinations

    print('Main Function DONE')

end


--math.randomseed(os.time())
math.randomseed(1234)










-- ----------------- SIMULATION CONFIGURATION ----------------------------

-- We're going to make things a bit cleaner by making all user-accessible 
--   properties be defined within a configuration file. These properties can 
--   be accessed from the 'myconf' structure in config.lua
-- Change the path to config.lua as appropriate
local file_dir = '/scratch/mveres/grasping/lib/lua_config.lua'
-- !!!!!
package.path = package.path .. ';' .. file_dir
require 'config'
-- .....
if myconf == nil then 
    print('UNABLE TO LOAD CONFIGURATION. CHECK FILE_DIR IS CORRECT.')
    simStopSimulation()
end

--"
-- We also need to specify where the file containing grasp candidates to test
--   is located. If not specified, we use the default file in config.lua. 
--   Specifying a file via command line is generally used for running parallel
-- Note that extra arguments can be passed in by changing the 'app_arg' #

--This is where we set up the path configurtaion for our grasps information (from Graspit!)
--Check in their config file to see how there set up these params and what is the form of their grasp candidates
--We do not do parallel running first, just run one by one manually
--"

local param_file = simGetStringParameter(sim_stringparam_app_arg1)

if param_file == '' then 
    input_file = myconf.data_dir .. myconf.object_file
else 
    input_file  = myconf.data_dir .. param_file
end

local idx_low = simGetStringParameter(sim_stringparam_app_arg2)
local idx_high = simGetStringParameter(sim_stringparam_app_arg3)


-- ----------------- LOADING GRASP CANDIDATES ----------------------------
--"
-- All of the grasp candidates are stored as a csv text file, but Lua is
--   terrible at processing these directly. We're going to read in the data,
--   and return structures carrying all relevant components
--"
print('Loading grasps from Graspit!...')
local siminfo = {}
-- Either read certain lines of a file, or the whole file itself
if idx_low ~= '' and idx_high ~= '' then
    print('  using indices: '..idx_low..'-'..idx_high)
    siminfo = csvfile_read_idx(input_file, ',', true, idx_low, idx_high)
else
    siminfo = csvfile_read(input_file, ',', true)
end
print('Grasps from Graspit! loaded')

if siminfo == nil then 
    print('UNABLE TO READ GRASP CANDIDATE FILE. CHECK THAT IT EXISTS.')
    simStopSimulation()
end

--This part needs to be changed accordingly
-- global variables (prefixed with "gl")
local data = parse_data(siminfo)
gl_com = data['obj_com']
gl_inertia = data['obj_inertia']
gl_obj_pose = data['obj_mats']
gl_obj_names = data['obj_names'] 
gl_grasp_candidates = data['angles']
gl_grasp_pose = data['gripper_mats']
gl_grasp_nums = data['grasp_nums']
gl_palm_dist = myconf.palm_distances
gl_vel = myconf.maxVel
gl_accel = myconf.maxAccel
gl_jerk = myconf.maxJerk


-- -- Each simulation is designed to collect data for a single object at a time.
-- --   This was done to avoid having constantly opening & closing object files,
-- --   (which creates more traffic on the network), or loading all into memory 
-- --   at once (which, if running sims in parallel, is redundant).

local object_name = gl_obj_names[1]:split(".obj")[1]
-- local start_num = gl_grasp_nums[1] -- Used so we don't overwrite files
-- local save_dir = myconf.working_dir..object_name..myconf.path_sep

-- r, _ = os.rename(myconf.working_dir, myconf.working_dir)
-- if r == nil then
--     os.execute("mkdir " .. myconf.working_dir)
-- end

-- -- Make sure the directory we want to save to exists, then open the data file
-- r, _ = os.rename(save_dir, save_dir)
-- if r == nil then
--     os.execute("mkdir " .. save_dir)
-- end



print('\n--------------------------------------------')
print('Using config: '..myconf.config)
print('Using path seperator: '..myconf.path_sep)
print('Using data directory: '..myconf.data_dir)
print('Using mesh directory: '..myconf.mesh_dir)
print('Using working directory: '..myconf.working_dir)
print('Using object file: '..object_name)
print('Using input from: '..input_file)
-- print('Using save directory: '..save_dir)
print('--------------------------------------------\n')

-- --------------- GET ALL RELEVANT OBJECT HANDLES --------------------------

-- Object handles for specific scene components. 
-- All object handles will be prefixed with a 'h_' marker.

--where these names come from???
h_camera_mask = simGetObjectHandle('kinect_mask')
h_camera_colour = simGetObjectHandle('kinect_rgb')
h_camera_depth = simGetObjectHandle('kinect_depth')
h_camera_dummy = simGetObjectHandle('camera_dummy') 
h_table_object = simGetObjectHandle('customizableTable_tableTop')
h_gripper_dummy = simGetObjectHandle('gripper_dummy')
h_gripper_palm = simGetObjectHandle(myconf.gripper_palm)
h_gripper_object = simGetObjectHandle(myconf.gripper_base)
h_gripper_proxim = simGetObjectHandle(myconf.contact_proximity_sensor)


-- The barrett hand can be swapped out with a manipulator with any number of 
--   fingers, so long as the appropriate locations in the config file are set.


--change all these config files to fecth gripper
--But before that, just test using barrett first, do not touch these first
h_gripper_contacts = {}
for _, fingertip in pairs(myconf.gripper_contacts) do 
    table.insert(h_gripper_contacts, simGetObjectHandle(fingertip))
end

h_gripper_sensors = {}
for _, force_sensor in pairs(myconf.gripper_force_sensors) do 
    table.insert(h_gripper_sensors, simGetObjectHandle(force_sensor))
end

if #h_gripper_contacts ~= #h_gripper_sensors then
    print('ERROR: #GRIPPER_CONTACTS != #GRIPPER_FORCE_SENSORS')
    simStopSimulation()
end


-- -------------------- SIMULATION CONSTANTS ----------------------------

-- The following contstants are bit-level flags used to control gripper
--   properties. Note that combinations set the WHOLE mask, which may influence
--   other properties. Check the API for a full list of possible flags.


--do not quite understand what they are doing here
OBJECT_PROP_RESPONDABLE = 
    sim_objectspecialproperty_renderable +
    sim_objectspecialproperty_detectable_all
GRIPPER_PROP_STATIC =  
    sim_modelproperty_not_dynamic    + 
    sim_modelproperty_not_renderable +
    sim_modelproperty_not_respondable
GRIPPER_PROP_VISIBLE = 
    sim_modelproperty_not_renderable + 
    sim_modelproperty_not_measurable 
GRIPPER_PROP_INVISIBLE = 
    sim_modelproperty_not_collidable + 
    sim_modelproperty_not_renderable + 
    sim_modelproperty_not_visible    + 
    sim_modelproperty_not_respondable+ 
    sim_modelproperty_not_dynamic


-- Record initial gripper configuration so it can be reset for each trial.

GRIPPER_COMPONENTS = simGetObjectsInTree(h_gripper_object)
GRIPPER_CONFIGURATION = simGetConfigurationTree(h_gripper_object)
GRIPPER_FINGER_ANGLES = myconf.gripper_finger_angles
MESH_FILE = myconf.mesh_dir .. object_name .. '.stl'

--do not quite understand what they are doing here

-- ------------------- VISUALIZATION PARAMETERS -----------------------------

-- In V-REP, there are a few useful modules for drawing to screen, which make 
--   things like debugging much easier.

black={0,0,0}
purple={1,0,1}
lightBlue={0,1,1}

-- Used for plotting spherical-type objects
sphereContainer = simAddDrawingObject(
    sim_drawing_spherepoints, myconf.display_point_size, 
    myconf.display_point_density, -1, myconf.display_num_points,
    black, black, black, light_blue)
    
-- Used for plotting line-type objects
lineContainer = simAddDrawingObject(
    sim_drawing_lines, myconf.display_vector_width, 
    myconf.display_point_density, -1, myconf.display_num_points,
    black, black, black, purple)

-- Reset the containers so they hold no elements
simAddDrawingObjectItem(sphereContainer, nil) 
simAddDrawingObjectItem(lineContainer, nil)


-- ------------------ CONFIGURE OBJECT PROPERTIES ---------------------------

-- We're only going to load a single object while running a sim, which allows 
--   us to avoid having to keep accessing the object file during simulation.
--   This also lets us set intrinsic object properties only once
-- We'll set the object to be respondable (i.e. when it comes into contact 
--   with the hand or table, it reacts) and detectable/renderable (for the 
--   various sensors within the sim)

print('mesh file: ',MESH_FILE)
vertices, indices, _, mesh_names = simImportMesh(4, MESH_FILE, 0, 0.001, 1.0)

h_object = simCreateMeshShape(0, 0, vertices[1], indices[1])

if h_object == nil then 
    print('ERROR: UNABLE TO CREATE MESH SHAPE')
    simStopSimulation() 
end

simSetObjectName(h_object,'object')
simSetShapeMaterial(h_object, simGetMaterialId(myconf.object_material))

-- Set the starting object position 
simReorientShapeBoundingBox(h_object, -1)
simSetObjectMatrix(h_object, h_workspace, gl_obj_pose[1])
simSetShapeMassAndInertia(h_object, 1, gl_inertia[1], gl_com[1])

-- Want to make the "generated" object be respondable
simSetObjectInt32Parameter(h_object, sim_shapeintparam_respondable, 1)

-- Want the object to be renderable by cams, and detectable by e.g. proxim
simSetObjectSpecialProperty(
    h_object, sim_objectspecialproperty_renderable +
    sim_objectspecialproperty_detectable_all) 
    

-- -- -------------------------- CAMERA PARAMETERS ---------------------------

-- -- We're going to set each camera within the scene to have the same parameters,
-- --   even though they capture different properties (e.g. RGB vs depth).

-- for _, cam in pairs({h_camera_depth, h_camera_colour, h_camera_mask}) do 
--     simSetObjectFloatParameter(
--         cam, sim_visionfloatparam_near_clipping, myconf.camera_near_clip)
--     simSetObjectFloatParameter(
--         cam, sim_visionfloatparam_far_clipping,  myconf.camera_far_clip)
--     simSetObjectFloatParameter(
--         cam, sim_visionfloatparam_perspective_angle, myconf.camera_fov)
--     simSetObjectInt32Parameter(
--         cam, sim_visionintparam_resolution_x, myconf.camera_resolution_x)
--     simSetObjectInt32Parameter(
--         cam, sim_visionintparam_resolution_y, myconf.camera_resolution_y)
-- end

-- -- Set the objects we want the camera to focus on (-1 = all, mask = object)
-- simSetObjectInt32Parameter(
--     h_camera_depth, sim_visionintparam_entity_to_render, -1)
-- simSetObjectInt32Parameter(
--     h_camera_colour, sim_visionintparam_entity_to_render, -1)
-- simSetObjectInt32Parameter(
--     h_camera_mask, sim_visionintparam_entity_to_render, h_object)


-- ------------- SET THE INITIAL GRIPPER POSITION ------------------------

-- Set the gripper state to be invisble, so we avoid any collisions at start
simSetModelProperty(h_gripper_object, GRIPPER_PROP_INVISIBLE)

-- Set initial gripper along z-axis of object, and point towards center
-- We get the dimensions of the bounding box of the object (aligned to world)
--   and set the starting location as 0.5*sqrt(x^2+y^2+z^2) along objects z

local bbox = getObjectBoxPositions(h_object)
local dim = getObjectBoxDimensions(bbox)
local workspace_radius = 0.5*math.sqrt( dim[1]^2+dim[2]^2+dim[3]^2)
simSetObjectPosition(h_gripper_proxim, h_object, {0,0,workspace_radius/2})

local object_position = simGetObjectPosition(h_object, -1)
pointObjectTowardsLocation(h_gripper_proxim, object_position, {0,0,1})  
moveGripper(h_gripper_object, h_object, h_gripper_proxim)

gl_gripper_mtx = simGetObjectMatrix(h_gripper_palm, h_object)


res,err=xpcall(threadFunction,function(err) return debug.traceback(err) end)
if not res then
    print('Error: ', err)
    simAddStatusbarMessage('Lua runtime error: '..err)
end


print('Done simulation!')


















