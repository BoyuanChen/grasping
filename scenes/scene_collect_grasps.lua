-- See: https://gist.github.com/walterlua/978161
--Set up the table
table.copy = function( t, ... )
    local copyShallow = function( src, dst, dstStart )
        local result = dst or {}
        local resultStart = 0
        if dst and dstStart then
            resultStart = dstStart
        end
        local resultLen = 0
        if "table" == type( src ) then
            resultLen = #src
            for i=1,resultLen do
                local value = src[i]
                if nil ~= value then
                    result[i + resultStart] = value
                else
                    resultLen = i - 1
                    break;
                end
            end
        end
        return result,resultLen
    end

    local result, resultStart = copyShallow( t )

    local srcs = { ... }
    for i=1,#srcs do
        local _,len = copyShallow( srcs[i], result, resultStart )
        resultStart = resultStart + len
    end
    
    return result
end

---------------------------------------------------------------------
-- See: http://nocurve.com/2014/03/05/simple-csv-read-and-write-using-lua/
local function csvfile_split(str, sep)
    sep = sep or ','
    fields={}
    local matchfunc = string.gmatch(str, "([^"..sep.."]+)")
    if not matchfunc then return {str} end
    for str in matchfunc do
        table.insert(fields, str)
    end
    return fields
end

---------------------------------------------------------------------
-- See: http://nocurve.com/2014/03/05/simple-csv-read-and-write-using-lua/
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


-- Modified from above
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
---------------------------------------------------------------------
-- See: http://nocurve.com/2014/03/05/simple-csv-read-and-write-using-lua/
function csvfile_write(file, data, sep)

    sep = sep or ','
    
    for i=1,#data do
        file:write(data[i])
        file:write(sep)
    end
    file:write('\n')
    file:flush()
end

---------------------------------------------------------------------
-- See: http://nocurve.com/2014/03/05/simple-csv-read-and-write-using-lua/
function string:split( inSplitPattern, outResults )
 
   if not outResults then
      outResults = {}
   end
   local start = 1
   local theSplitStart, theSplitEnd = string.find(self, inSplitPattern, start)
   while theSplitStart do
      table.insert( outResults, string.sub( self, start, theSplitStart-1))
      start = theSplitEnd + 1
      theSplitStart, theSplitEnd = string.find( self, inSplitPattern, start)
   end
   table.insert( outResults, string.sub(self, start))
   return outResults
end


---------------------------------------------------------------------

-- Gets the bounding box components of an object
function getObjectBoxPositions(object_handle)

    local r, min_x = simGetObjectFloatParameter(
		object_handle, sim_objfloatparam_objbbox_min_x)
    local r, max_x = simGetObjectFloatParameter(
		object_handle, sim_objfloatparam_objbbox_max_x)
    local r, min_y = simGetObjectFloatParameter(
		object_handle, sim_objfloatparam_objbbox_min_y)
    local r, max_y = simGetObjectFloatParameter(
		object_handle, sim_objfloatparam_objbbox_max_y)
    local r, min_z = simGetObjectFloatParameter(
		object_handle, sim_objfloatparam_objbbox_min_z)
    local r, max_z = simGetObjectFloatParameter(
		object_handle, sim_objfloatparam_objbbox_max_z)

    return {min_x, max_x, min_y, max_y, min_z, max_z}
end


-- objectBoxPositions = (min_x, max_x, min_y, max_y, min_z, max_z)
function getObjectBoxDimensions(objectBoxPositions)

    local dim ={
		math.abs(objectBoxPositions[2] - objectBoxPositions[1]),
        math.abs(objectBoxPositions[4] - objectBoxPositions[3]),
        math.abs(objectBoxPositions[5] - objectBoxPositions[6])}
    return dim
end


-- Checks that the grasp is valid
--     Looking for the right object being grasped, and no 
--     collisions/wrong number of contacts)

--This is the place to find if you really want to rewrite the functions in Graspit! into Vrep
function checkContacts(contactPoints)

    local result = 1
    local info = {}
	local na = {'nil', 'nil', 'nil'} 
	local bad_info = {{na, na, na},{na, na, na},{na, na, na}}

    for i = 1, #contactPoints, 1 do
		
		-- h = handles of colliding objects
		local h, position, force, normal = simGetContactInfo(
			sim_handle_all,contactPoints[i],1+sim_handleflag_extended)
        
        -- Check if the contact point is in contact with the object
        if (h == nil) or (h[1] ~= contactPoints[i]) or (h[2] ~= h_object) then
			return nil, bad_info
		else
            table.insert(info, {position, force, normal})
        end
    end

    return result, info
end


function skewSymmetric(x, y, z)
    local matrix = require 'matrix'
    return matrix{{0,-z,y}, {z,0,-x},{-y,x,0}}
end


-- Calculates the rotation needed to bring two axes coincident
function getRotMat(vector1, vector2)

    -- Use the matrix library from https://github.com/davidm/lua-matrix
    -- Install by placing "matrix.lua" and "complex.lua" into the lua folder,
	-- e.g.: C:\Program Files (x86)\V-REP3\V-REP_PRO_EDU\lua
    local matrix = require 'matrix'

    -- need to store vectors as matrices for library
    local vec1 = matrix{vector1}^'T'
    local vec2 = matrix{vector2}^'T'
    
    -- Normalize the vectors to be unit vectors
    local a = matrix.divnum(vec1, matrix.normf(vec1))
    local b = matrix.divnum(vec2, matrix.normf(vec2))

    -- Take the vector crossproduct
    local v = matrix.cross(a,b)

    -- Normalize the cross product, then take the inner dot-product
    local s = matrix.normf(v)
    local c = a:scalar(b) 

    -- Build the 3x3 identity matrix and skew symmetric matrices
    local eye = matrix{ {1,0,0},{0,1,0},{0,0,1}}
    local sk = skewSymmetric(v[1][1], v[2][1], v[3][1])
    local sk_squared = matrix.mul(sk, sk)

    -- Get the rotation matrix
    local p1 = matrix.add(eye, sk)
    local p2 = matrix.mulnum(sk_squared, (1.0-c)/(s^2))
    local Rmat = matrix.add(p1, p2)

    -- The 0's represent position (relative to itself)
    local arrayMat = {Rmat[1][1],Rmat[1][2],Rmat[1][3], 0,
                      Rmat[2][1],Rmat[2][2],Rmat[2][3], 0,
                      Rmat[3][1],Rmat[3][2],Rmat[3][3], 0}

    return arrayMat
end


-- Orients the camera 'z' angle towards a point in 3D space
-- P2, P3 are vectors
function pointObjectTowardsLocation(objectToAdjust, pointToLookAt, pointToOrientXTowards)

	local matrix = require 'matrix'

	-- pointToLookAt is a direction vector that has translation and rotation
	local objMat  = simGetObjectMatrix(objectToAdjust, -1)
	local objMatInv  = simGetInvertedMatrix(objMat)

	-- pointToOrientXTowards is a direction vector that has rotation
	local orient = simGetObjectOrientation(objectToAdjust, -1)
	local rotMat  = simBuildMatrix({0,0,0}, orient)
	local rotMatInv = simGetInvertedMatrix(rotMat)

	-- Calculate direction vector (P2) and Up vector (P3)
	local P1 = {0, 0, 0}
	local P2 = simMultiplyVector(objMatInv, pointToLookAt)
	local P3 = simMultiplyVector(rotMatInv, pointToOrientXTowards)
	
	-- Calculate an "Up" direction vector that points 'Y' towards origin
	-- NOTE: See http://stackoverflow.com/questions/14250208/three-js-how-to-rotate-an-object-to-lookat-one-point-and-orient-towards-anothe
	local D = matrix{P2}^'T' 
    local U = matrix{P3}^'T'
	local D_norm = matrix.divnum(D, matrix.normf(D))

	local right = matrix.cross(U, D_norm)
	local right_norm = matrix.divnum(right, matrix.normf(right))
	local backwards = matrix.cross(right_norm, U)
	local backwards_norm = matrix.divnum(backwards, matrix.normf(backwards))
	local up = matrix.cross(backwards_norm, right_norm)
	local up_norm = matrix.divnum(up, matrix.normf(up))

	-- This is the rotation matrix that aligns the up-vector
	local rotMat = {right_norm[1][1], up_norm[1][1], backwards_norm[1][1], 0,
					right_norm[2][1], up_norm[2][1], backwards_norm[2][1], 0,
					right_norm[3][1], up_norm[3][1], backwards_norm[3][1], 0}
	local eulerFromMat = simGetEulerAnglesFromMatrix(rotMat)
    simSetObjectOrientation(objectToAdjust, objectToAdjust, eulerFromMat)
		
	-- Calculate rotation matrix to point camera towards object
	-- Need to find direction vector from new frame to object
	local objMat       = simGetObjectMatrix(objectToAdjust, -1)
	local objMatInv    = simGetInvertedMatrix(objMat)
	local directionVec = simMultiplyVector(objMatInv, pointToLookAt)
	
    local rotMat = getRotMat({0,0,1}, directionVec)
    local eulerFromMat = simGetEulerAnglesFromMatrix(rotMat)
    simSetObjectOrientation(objectToAdjust, objectToAdjust, eulerFromMat)
	
	local p = simGetObjectPosition(h_camera_dummy, -1)
    local line={p[1], p[2], p[3], 
                pointToLookAt[1], pointToLookAt[2], pointToLookAt[3]}
    simAddDrawingObjectItem(lineContainer, line)
end


-- This function gets the position and orientation of each components in the gripper,
-- relative to some given object ("wrtObjectHandle")
function getGraspInformation(inContact, contactsInfo, object_tree, wrtObjectHandle)

    -- Names: Keeps track of object names (+ number of items)
    -- Configuration: Keeps track of object-specific properties
    local names = {}
    local configurations = {}

    if inContact == nil then inContact={0} else inContact={inContact} end

    names = table.copy(names, {'AllTipsInContact', 1})
    configurations = table.copy(configurations, inContact)

    -- Should have the contact info already calculated from our previous checks
	for i =1, #contactsInfo, 1 do

        -- Save information about the name, then number of items associated with it
        -- **NOTE**: These are WRT world frame
        local r, forceVector, torqueVector = simReadForceSensor(h_gripper_sensors[i])
		
		if forceVector  == nil then forceVector  = {-1, -1, -1} end
		if torqueVector == nil then torqueVector = {-1, -1, -1} end

        names = table.copy(names, 
			{'contactPoint'..(i-1), #contactsInfo[i][1],
			 'contactForce'..(i-1), #contactsInfo[i][2], 
			 'contactNormal'..(i-1),#contactsInfo[i][3],
			 'forceSensor'..(i-1),  #forceVector,
			 'torqueSensor'..(i-1), #torqueVector,
			 'forceSensorStatus'..(i-1), 1,
			 'torqueSensorStatus'..(i-1), 1})
								   
        configurations = table.copy(configurations, 
			contactsInfo[i][1], contactsInfo[i][2], 
			contactsInfo[i][3], forceVector, torqueVector, {r}, {r})
    end
	
	
	local collision = 0
	
    -- For each item in the gripper, get its current configuration (3x4 matrix)
    for i =1, #object_tree, 1 do

        local name = simGetObjectName(object_tree[i])
        if string.find(name, 'respondable') ~= nil then

            local object_matrix=simGetObjectMatrix(object_tree[i], wrtObjectHandle)
            names = table.copy(names, {name, #object_matrix})
            configurations = table.copy(configurations, object_matrix)

            -- Check if we're colliding with anything but the fingertips
            local check_collision = simCheckCollision(object_tree[i], h_object)
            if check_collision == 1 and string.find(name, 'fingerTip')==nil then 
                collision = collision + check_collision 
                print('Colliding with:' , name)
            end
        end
        if string.find(name, 'joint') ~= nil then
        
            local intrinsic_angle = simGetJointPosition(object_tree[i])
            names = table.copy(names, {name..'_pos',1})
            configurations = table.copy(configurations, {intrinsic_angle})

        end
    end
		
	local object_matrix=simGetObjectMatrix(h_gripper_palm, wrtObjectHandle)
	names = table.copy(names, {'BarrettHand_PACF', #object_matrix})
	configurations = table.copy(configurations, object_matrix)
	
	names = table.copy(names, {'NumDiffObjectsColliding', 1})
    configurations = table.copy(configurations, {collision})

    local wrtObjectMatrix = simGetObjectMatrix(wrtObjectHandle, -1)
    local cameraMatrix = simGetObjectMatrix(h_camera_dummy, wrtObjectHandle)
    local contactMatrix = simGetObjectMatrix(h_gripper_proxim, wrtObjectHandle)

    -- Also save the mass, inertia, and center of mass of the object
    local mass, inertia, com = simGetShapeMassAndInertia(h_object, wrtObjectMatrix)
    local material_id  = simGetShapeMaterial(h_object)

    local fingerAngle = simGetScriptSimulationParameter(sim_handle_all, 'fingerAngle')
    local object_matrix = simGetObjectMatrix(h_object, wrtObjectHandle)

    local gripper_euler = simGetEulerAnglesFromMatrix(gl_gripper_mtx)
	
    local gripper_pos = 
		{gl_gripper_mtx[4], gl_gripper_mtx[8], gl_gripper_mtx[12]}
		
    local prerot_euler = 
		{gl_grasp_candidates[attempt][1], 
		 gl_grasp_candidates[attempt][2], 
		 gl_grasp_candidates[attempt][3]}
							
	local postrot_euler = 
		{gl_grasp_candidates[attempt][4], 
	     gl_grasp_candidates[attempt][5], 
	     gl_grasp_candidates[attempt][6]}
	
	-- returned in absolute coordinate
	local linVelocity, angVelocity = simGetObjectVelocity(h_object)
	local surface_point = point_wrt_object

    -- Add a "header" telling us how much information is in each component
    names = table.copy(names, 
		{'wrtObjectMatrix',#wrtObjectMatrix},
		{'cameraMatrix', #cameraMatrix},
		{'contactMatrix', #contactMatrix},
		{'object_matrix',#object_matrix},
        {'rot_invariant_matrix', #rot_invariant_matrix},
        {'rot_variant_matrix', #rot_variant_matrix},
		{'mass',1},
		{'inertia',#inertia},
		{'com',#com},
		{'fingerAngle', #fingerAngle},
		{'material_id',1},
		{'objectLinearVelocity', #linVelocity},
		{'objectAngularVelocity',#angVelocity},
		{'gripper_inital_euler', #gripper_euler},
		{'gripper_inital_pos', #gripper_pos},
		{'prerot_euler', #prerot_euler},
		{'postrot_euler', #postrot_euler},
		{'surface_point_wrt_object',#surface_point},
		{'camera_resolution_x',1},
		{'camera_resolution_y',1},
		{'camera_near_clip',1},
		{'camera_far_clip',1},
		{'camera_fov',1},
		{'camera_contact_offset',1}					  
		)
	
    -- Add all the data into a single table
    configurations = table.copy(configurations, 
		wrtObjectMatrix, 
		cameraMatrix, 
		contactMatrix, 
		object_matrix, 
        rot_invariant_matrix,
        rot_variant_matrix,
		{mass},  
		inertia, 
		com, 
		fingerAngle,
		{material_id}, 
		linVelocity, 
		angVelocity, 
		gripper_euler,
		gripper_pos, 
		prerot_euler, 
		postrot_euler,
		surface_point,
		{myconf.camera_resolution_x}, 
		{myconf.camera_resolution_y}, 
		{myconf.camera_near_clip},
		{myconf.camera_far_clip}, {myconf.camera_fov}, 
		{myconf.camera_contact_offset})

	-- Quick check that len(names) == len(configuration)
	--sum = 0
    --for i = 2, #names, 2 do sum = sum + names[i] end
    --print('Sum: '..sum..' Config: '..#configurations)
	--if sum ~= #configurations then	
	--	print('Mismatch in the number of elements being recorded. Try again')
	--	simStopSimulation()
	--end
	
    return {names, configurations}

end


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


-- Moves the gripper by moving the palm-attached coordinate frame
function moveGripper(gripper, wrtHandleLocation, dummy_location)

	-- Move the gripper to a given location
    local cm = simGetObjectMatrix(dummy_location, wrtHandleLocation)
	simSetObjectMatrix(gripper, wrtHandleLocation, cm)

	--Move the gripper back some distance WRT palm-attached coordinate frame
	local modelFrameRelativeToDummy=simGetObjectPosition(gripper, h_gripper_palm)
    simSetObjectPosition(gripper,gripper,modelFrameRelativeToDummy)
	    
end


function verifyContacts()

	-- Check whether gripper is interacting with table (yes? skip)
	local CONTACT_OK = true
	local total_contacts = 0
	local threshold = 50
	
	for k = 1, #GRIPPER_COMPONENTS, 1 do
	
		local tbl = simCheckCollisionEx(
			GRIPPER_COMPONENTS[k], h_table_object)
		local obj = simCheckCollisionEx(
			GRIPPER_COMPONENTS[k], h_object)
		local contact_obj = false
		
		total_contacts = total_contacts + tbl + obj

		-- Check if parts of the gripper (that are not fingertips)
		--   are in contact with object/table
		if tbl >0 or obj>0 then
			for l = 1, #h_gripper_contacts, 1 do
				if GRIPPER_COMPONENTS[k] == h_gripper_contacts[l] then
					contact_obj = true; break;
				end
			end
		end
		if contact_obj == false and total_contacts > threshold  then 
			CONTACT_OK = false; 
			break; 
		end
	end
	return CONTACT_OK

end


function calcSphericalPos(rad, theta, phi)
    
    local x = rad*math.cos(theta)*math.sin(phi)
    local y = rad*math.sin(theta)*math.sin(phi)
    local z = rad*math.cos(phi)
    
    return {x, y, z}

end


threadCameraFunction=function()

    -- Get initial camera images
    local h_topdown = simGetObjectHandle('topdown_dummy')
    local cm = simGetObjectMatrix(h_topdown, h_workspace)
    simSetObjectMatrix(h_camera_depth, h_workspace, cm)
    simSetObjectMatrix(h_camera_colour, h_workspace, cm)
    simSwitchThread()

    -- We only need a single picture of the object, so we need to 
    --   make sure that the simulation knows to render it now
    simHandleVisionSensor(h_camera_depth)
    simHandleVisionSensor(h_camera_colour)
    simSwitchThread()

    local topdown_depth_image = simGetVisionSensorDepthBuffer(h_camera_depth)
    local topdown_colour_image = simGetVisionSensorImage(h_camera_colour)
    local topdown_matrix = simGetObjectMatrix(h_topdown, h_workspace)
    simSwitchThread()

    csvfile_write(savefile, table.copy({"TOPDOWN_DEPTH_0"}, topdown_depth_image))
    simSwitchThread()
    csvfile_write(savefile, table.copy({"TOPDOWN_COLOUR_0"}, topdown_colour_image))
    simSwitchThread()
    csvfile_write(savefile, table.copy({"TOPDOWN_MATRIX_0"}, topdown_matrix))
    simSwitchThread()

    local n_rots = 8
    for i = 1, n_rots, 1 do
	for j = 1, 2, 1 do
        local pos = calcSphericalPos(0.65, i*2*math.pi/n_rots, j*math.pi/6)
        local table_pos = simGetObjectPosition(h_table_object, -1)
        simSetObjectPosition(h_topdown, h_workspace, pos)
        pointObjectTowardsLocation(h_topdown, table_pos, {0,0,1})

        local cm = simGetObjectMatrix(h_topdown, h_workspace)
        simSetObjectMatrix(h_camera_depth, h_workspace, cm)
        simSetObjectMatrix(h_camera_colour, h_workspace, cm)
        simSwitchThread()

        -- We only need a single picture of the object, so we need to 
        --   make sure that the simulation knows to render it now
        simHandleVisionSensor(h_camera_depth)
        simHandleVisionSensor(h_camera_colour)
        simSwitchThread()
        local topdown_depth_image = simGetVisionSensorDepthBuffer(h_camera_depth)
        local topdown_colour_image = simGetVisionSensorImage(h_camera_colour)
        local topdown_matrix = simGetObjectMatrix(h_topdown, h_workspace)
        simSwitchThread()
        csvfile_write(savefile, table.copy({"TOPDOWN_DEPTH_"..(i+j)}, topdown_depth_image))
        simSwitchThread()
        csvfile_write(savefile, table.copy({"TOPDOWN_COLOUR_"..(i+j)}, topdown_colour_image))
        simSwitchThread()
        csvfile_write(savefile, table.copy({"TOPDOWN_MATRIX_"..(i+j)}, topdown_matrix))
        simSwitchThread()
    end
	end
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
				
				-- ----------------- COLLECT CAMERA IMAGES -------------------
				
				-- Randomly set the colour of the object
				local randr = math.random()
				local randg = math.random()
				local randb = math.random()
				local colour = {randr, randg, randb}
				simSetShapeColor(
					h_object, nil, sim_colorcomponent_ambient_diffuse, colour)

				-- For positioning the camera, we're going to do something 
				--   similar to what we did for positioning the manipulator. 
				--   We start by setting the camera location equal to the grasp
				--   location, then moving it backwards by a given offset
				local contact_matrix = simGetObjectMatrix(h_gripper_dummy, h_workspace)
				simSetObjectMatrix(h_camera_dummy, h_workspace, contact_matrix)
				simSwitchThread()

				local camera_approach = 
					{dvec[1]*myconf.camera_contact_offset,
					 dvec[2]*myconf.camera_contact_offset,
					 dvec[3]*myconf.camera_contact_offset}
				simSetObjectPosition(h_camera_dummy, h_camera_dummy, camera_approach)
				simSwitchThread()

				-- Once we've positioned the camera, we need to make sure that 
				--   it points towards the detected surface point, while the 
				--   y-axis points upwards
				pointObjectTowardsLocation(h_camera_dummy, point_wrt_world, {0,0,1})
				simSwitchThread()



                -- Collect one camera image where we're looking at the object, but rotation
                -- invariant
				local cm = simGetObjectMatrix(h_camera_dummy, h_workspace)
				simSetObjectMatrix(h_camera_depth, h_workspace, cm)
				simSetObjectMatrix(h_camera_colour, h_workspace, cm)
				simSetObjectMatrix(h_camera_mask, h_workspace, cm)
				simSwitchThread()

				simHandleVisionSensor(h_camera_depth)
				simHandleVisionSensor(h_camera_colour)
				simHandleVisionSensor(h_camera_mask)
				simSwitchThread()

				local object_image = simGetVisionSensorDepthBuffer(h_camera_depth)
				local colour_image = simGetVisionSensorImage(h_camera_colour)
				local mask_image = simGetVisionSensorImage(h_camera_mask)
                rot_invariant_matrix = simGetObjectMatrix(h_camera_dummy, h_workspace)
				simSwitchThread()
                simWait(5)


                -- Collect one camera image where we're at the same orientation as the gripper
                local orient = simGetObjectOrientation(h_gripper_dummy, -1)
                simSetObjectOrientation(h_camera_dummy, -1, orient)

                local cm = simGetObjectMatrix(h_camera_dummy, h_workspace)
                simSetObjectMatrix(h_camera_depth, h_workspace, cm)
                simSetObjectMatrix(h_camera_colour, h_workspace, cm)
                simSwitchThread()

                simHandleVisionSensor(h_camera_depth)
                simHandleVisionSensor(h_camera_colour)
                simSwitchThread()

                local direct_depth_image = simGetVisionSensorDepthBuffer(h_camera_depth)
                local direct_colour_image = simGetVisionSensorImage(h_camera_colour)
                rot_variant_matrix = simGetObjectMatrix(h_camera_dummy, h_workspace)
                simSwitchThread()
                simWait(5)


				-- Visualizing the camera view direction with a line
				local cam_pt = simGetObjectPosition(h_camera_dummy, -1)
				simAddDrawingObjectItem(
					lineContainer, {point_wrt_world[1], point_wrt_world[2], 
					point_wrt_world[3], cam_pt[1], cam_pt[2], cam_pt[3]})


				-- -------------- ATTEMPT TO GRASP OBJECT --------------------
									
				-- Check that the gripper is curretly not contacting anything, 
				-- and that the camera is above the workspace (table) level
				is_no_contact = verifyContacts()
				local camera_wrt_workspace = simGetObjectPosition(h_camera_dummy, h_table_object)
				simSwitchThread()
				
				if is_no_contact == true and camera_wrt_workspace[3]>=0 then 
				
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
						
						-- Save the Pre-grasp data
						local pregrasp_info = getGraspInformation(
							r, contactsObject, GRIPPER_COMPONENTS, h_workspace)
						simSwitchThread()

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
                        csvfile_write(savefile, table.copy({"DIRECT_DEPTH"}, direct_depth_image))
                        simSwitchThread()
                        csvfile_write(savefile, table.copy({"DIRECT_COLOUR"}, direct_colour_image))
                        simSwitchThread()

						csvfile_write(savefile, 
							table.copy({"GRIPPER_IMAGE"}, object_image))
						simSwitchThread()
						
						csvfile_write(savefile, 
							table.copy({"GRIPPER_IMAGE_COLOUR"}, colour_image))
						simSwitchThread()
						
						csvfile_write(savefile, 
							table.copy({"GRIPPER_MASK_IMAGE"}, mask_image))
						simSwitchThread()
						
						csvfile_write(savefile, 
							table.copy({"GRIPPER_HEADER"}, pregrasp_info[1]))
						simSwitchThread()
						
						csvfile_write(savefile, 
							table.copy({"GRIPPER_PREGRASP"}, pregrasp_info[2]))
						simSwitchThread()
						
						csvfile_write(savefile, 
							table.copy({"GRIPPER_POSTGRASP"}, info[2]))
						simSwitchThread()
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

    print('DONE')

end


--math.randomseed(os.time())
math.randomseed(1234)


-- ----------------- SIMULATION CONFIGURATION ----------------------------

-- We're going to make things a bit cleaner by making all user-accessible 
--   properties be defined within a configuration file. These properties can 
--   be accessed from the 'myconf' structure in config.lua
-- Change the path to config.lua as appropriate

--local file_dir = 'C:/Users/Matt/Documents/grasping/lib/lua_config.lua'
local file_dir = '/scratch/mveres/grasping/lib/lua_config.lua'
--local file_dir = '/home/robot/Documents/grasping/lib/lua_config.lua'

package.path = package.path .. ';' .. file_dir
require 'config'
if myconf == nil then 
	print('UNABLE TO LOAD CONFIGURATION. CHECK FILE_DIR IS CORRECT.')
	simStopSimulation()
end

-- We also need to specify where the file containing grasp candidates to test
--   is located. If not specified, we use the default file in config.lua. 
--   Specifying a file via command line is generally used for running parallel
-- Note that extra arguments can be passed in by changing the 'app_arg' #

local param_file = simGetStringParameter(sim_stringparam_app_arg1)

if param_file == '' then 
	input_file = myconf.data_dir .. myconf.object_file
else 
	input_file  = myconf.data_dir .. param_file
end

local idx_low = simGetStringParameter(sim_stringparam_app_arg2)
local idx_high = simGetStringParameter(sim_stringparam_app_arg3)

-- ----------------- LOADING GRASP CANDIDATES ----------------------------

-- All of the grasp candidates are stored as a csv text file, but Lua is
--   terrible at processing these directly. We're going to read in the data,
--   and return structures carrying all relevant components

print('Loading simulation info')
local siminfo = {}
-- Either read certain lines of a file, or the whole file itself
if idx_low ~= '' and idx_high ~= '' then
	print('  using indices: '..idx_low..'-'..idx_high)
	siminfo = csvfile_read_idx(input_file, ',', true, idx_low, idx_high)
else
	siminfo = csvfile_read(input_file, ',', true)
end
print('Done loading simulation info')

if siminfo == nil then 
	print('UNABLE TO READ GRASP CANDIDATE FILE. CHECK THAT IT EXISTS.')
	simStopSimulation()
end

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

-- Each simulation is designed to collect data for a single object at a time.
--   This was done to avoid having constantly openenin & closing object files,
--   (which creates more traffic on the network), or loading all into memory 
--   at once (which, if running sims in parallel, is redundant).

local object_name = gl_obj_names[1]:split(".obj")[1]
local start_num = gl_grasp_nums[1] -- Used so we don't overwrite files
local save_dir = myconf.working_dir..object_name..myconf.path_sep

r, _ = os.rename(myconf.working_dir, myconf.working_dir)
if r == nil then
	os.execute("mkdir " .. myconf.working_dir)
end

-- Make sure the directory we want to save to exists, then open the data file
r, _ = os.rename(save_dir, save_dir)
if r == nil then
	os.execute("mkdir " .. save_dir)
end



print('\n--------------------------------------------')
print('Using config: '..myconf.config)
print('Using path seperator: '..myconf.path_sep)
print('Using data directory: '..myconf.data_dir)
print('Using mesh directory: '..myconf.mesh_dir)
print('Using working directory: '..myconf.working_dir)
print('Using object file: '..object_name)
print('Using input from: '..input_file)
print('Using save directory: '..save_dir)
print('--------------------------------------------\n')

-- --------------- GET ALL RELEVANT OBJECT HANDLES --------------------------

-- Object handles for specific scene components. 
-- All object handles will be prefixed with a 'h_' marker.

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


-- All info will be collected relative to this "workspace" location.

h_workspace = simCreateDummy(0.01)
simSetObjectName(h_workspace, 'workspace')
simSetObjectMatrix(h_workspace, -1, simGetObjectMatrix(h_table_object, -1))


-- -------------------- SIMULATION CONSTANTS ----------------------------

-- The following contstants are bit-level flags used to control gripper
--   properties. Note that combinations set the WHOLE mask, which may influence
--   other properties. Check the API for a full list of possible flags.

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

	
-- -------------------------- CAMERA PARAMETERS ---------------------------

-- We're going to set each camera within the scene to have the same parameters,
--   even though they capture different properties (e.g. RGB vs depth).

for _, cam in pairs({h_camera_depth, h_camera_colour, h_camera_mask}) do 
	simSetObjectFloatParameter(
		cam, sim_visionfloatparam_near_clipping, myconf.camera_near_clip)
	simSetObjectFloatParameter(
		cam, sim_visionfloatparam_far_clipping,  myconf.camera_far_clip)
	simSetObjectFloatParameter(
		cam, sim_visionfloatparam_perspective_angle, myconf.camera_fov)
	simSetObjectInt32Parameter(
		cam, sim_visionintparam_resolution_x, myconf.camera_resolution_x)
	simSetObjectInt32Parameter(
		cam, sim_visionintparam_resolution_y, myconf.camera_resolution_y)
end

-- Set the objects we want the camera to focus on (-1 = all, mask = object)
simSetObjectInt32Parameter(
	h_camera_depth, sim_visionintparam_entity_to_render, -1)
simSetObjectInt32Parameter(
	h_camera_colour, sim_visionintparam_entity_to_render, -1)
simSetObjectInt32Parameter(
	h_camera_mask, sim_visionintparam_entity_to_render, h_object)


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

-- ----------------- BEGIN DATA COLLECTION -------------------------------

local out_file = save_dir .. object_name .. '_' .. start_num .. '.txt'
savefile = assert(io.open(out_file, 'w'))


-- Collect camera images once for main scene
res,err=xpcall(threadCameraFunction,function(err) return debug.traceback(err) end)
if not res then
	print('Error: ', err)
	simAddStatusbarMessage('Lua runtime error: '..err)
end

-- Here we execute the regular thread code:
res,err=xpcall(threadFunction,function(err) return debug.traceback(err) end)
if not res then
	print('Error: ', err)
	simAddStatusbarMessage('Lua runtime error: '..err)
end

savefile:close()

print('Done simulation!')
if myconf.config == 'gpu' and param_file ~= '' then
	os.exit()
end
